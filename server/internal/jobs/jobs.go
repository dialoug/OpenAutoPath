// Package jobs runs each immutable model request in a separate working directory.
package jobs

import (
	"bufio"
	"context"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"slices"
	"sort"
	"sync"
	"time"
)

type Request struct {
	AutoTool        bool            `json:"autoTool,omitempty"`
	CompletedMoves  *int            `json:"completedMoves,omitempty"`
	StockID         string          `json:"-"`
	StockFile       string          `json:"stockFile,omitempty"`
	StockName       string          `json:"stockName,omitempty"`
	StockHash       string          `json:"stockHash,omitempty"`
	ConfirmedSetups *[]string       `json:"confirmedSetups,omitempty"`
	Feature         string          `json:"feature,omitempty"`
	Strategy        string          `json:"strategy,omitempty"`
	Step            float64         `json:"step,omitempty"`
	Document        json.RawMessage `json:"document,omitempty"`
	Operation       string          `json:"operation,omitempty"`
	SourceName      string          `json:"sourceName,omitempty"`
	LibraryID       string          `json:"-"`
	LibraryFile     string          `json:"libraryFile,omitempty"`
	AppendID        string          `json:"-"`
	AppendFile      string          `json:"appendFile,omitempty"`
	Version         int             `json:"version"`
	Action          string          `json:"action"`
	Model           string          `json:"model"`
	Hole            string          `json:"hole,omitempty"`
	Tool            string          `json:"tool,omitempty"`
	Feed            float64         `json:"feed,omitempty"`
	RPM             float64         `json:"rpm,omitempty"`
}

type Job struct {
	SourceHash string    `json:"sourceHash,omitempty"`
	Files      []string  `json:"files,omitempty"`
	Name       string    `json:"name,omitempty"`
	Created    time.Time `json:"created,omitempty"`
	Stage      string    `json:"stage,omitempty"`
	ID         string    `json:"id"`
	ModelID    string    `json:"modelId"`
	Action     string    `json:"action"`
	State      string    `json:"state"`
	Error      string    `json:"error,omitempty"`
}

type Manager struct {
	root, worker string
	slots        chan struct{}
	admitted     chan struct{}
	mu           sync.Mutex
	active       map[string]context.CancelFunc
}

func Token() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	return hex.EncodeToString(b[:]), nil
}

var idPattern = regexp.MustCompile("^[0-9a-f]{32}$")
var ErrBusy = errors.New("CAM job queue is full")

func ValidID(id string) bool { return idPattern.MatchString(id) }

func New(root, worker string, limit int) (*Manager, error) {
	if limit < 1 {
		return nil, errors.New("worker limit must be positive")
	}
	root, err := filepath.Abs(root)
	if err != nil {
		return nil, err
	}
	worker, err = filepath.Abs(worker)
	if err != nil {
		return nil, err
	}
	if _, err = os.Stat(worker); err != nil {
		return nil, err
	}
	if err = os.MkdirAll(root, 0700); err != nil {
		return nil, err
	}
	return &Manager{
		root: root, worker: worker, slots: make(chan struct{}, limit),
		admitted: make(chan struct{}, limit+32), active: make(map[string]context.CancelFunc),
	}, nil
}

func (m *Manager) dir(owner, id string) string { return filepath.Join(m.root, owner, id) }
func (m *Manager) Load(owner, id string) (Job, error) {
	var j Job
	if !ValidID(owner) || !ValidID(id) {
		return j, os.ErrNotExist
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	data, err := os.ReadFile(filepath.Join(m.dir(owner, id), "job.json"))
	if err != nil {
		return j, err
	}
	if err = json.Unmarshal(data, &j); err != nil {
		return j, err
	}
	if m.active[id] == nil && (j.State == "queued" || j.State == "running") {
		j.State = "failed"
		j.Error = "Server restarted during this job"
	}
	return j, nil
}

func save(dir string, j Job) error {
	data, err := json.Marshal(j)
	if err != nil {
		return err
	}
	tmp := filepath.Join(dir, "job.tmp")
	if err = os.WriteFile(tmp, data, 0600); err != nil {
		return err
	}
	return os.Rename(tmp, filepath.Join(dir, "job.json"))
}

func (m *Manager) Submit(owner, modelID string, req Request, upload io.Reader) (Job, error) {
	if !ValidID(owner) {
		return Job{}, os.ErrPermission
	}
	select {
	case m.admitted <- struct{}{}:
	default:
		return Job{}, ErrBusy
	}
	submitted := false
	defer func() {
		if !submitted {
			<-m.admitted
		}
	}()
	id, err := Token()
	if err != nil {
		return Job{}, err
	}
	if upload != nil {
		modelID = id
	} else if req.Action != "library" {
		model, err := m.Load(owner, modelID)
		if err != nil || model.State != "done" || model.Action != "analyze" {
			return Job{}, os.ErrNotExist
		}
		req.SourceName = model.Name
	}
	if req.StockID != "" {
		asset, e := m.Load(owner, req.StockID)
		if e != nil || asset.State != "done" || asset.Action != "analyze" || req.Action != "stock" {
			return Job{}, os.ErrNotExist
		}
		req.StockFile = filepath.Join(m.dir(owner, asset.ID), "model.step")
		req.StockName = asset.Name
		req.StockHash = asset.SourceHash
		if req.StockHash == "" {
			// Backfill models uploaded before source hashes were recorded.
			f, e := os.Open(req.StockFile)
			if e != nil {
				return Job{}, e
			}
			h := sha256.New()
			_, e = io.Copy(h, f)
			f.Close()
			if e != nil {
				return Job{}, e
			}
			asset.SourceHash = hex.EncodeToString(h.Sum(nil))
			m.mu.Lock()
			e = save(m.dir(owner, asset.ID), asset)
			m.mu.Unlock()
			if e != nil {
				return Job{}, e
			}
			req.StockHash = asset.SourceHash
		}
	} else if len(req.Document) > 0 && req.Action != "project" && req.Action != "library" {
		var ref struct {
			Job struct {
				Stock struct {
					Kind int    `json:"source_kind"`
					Hash string `json:"source_fingerprint"`
				}
			}
		}
		if json.Unmarshal(req.Document, &ref) == nil && ref.Job.Stock.Kind == 2 && ref.Job.Stock.Hash != "" {
			assets, e := m.List(owner)
			if e != nil {
				return Job{}, e
			}
			for _, asset := range assets {
				if asset.Action == "analyze" && asset.SourceHash == ref.Job.Stock.Hash {
					req.StockFile = filepath.Join(m.dir(owner, asset.ID), "model.step")
					break
				}
			}
		}
	}
	if req.LibraryID != "" {
		req.LibraryFile, err = m.Artifact(owner, req.LibraryID, "library.oaptl")
		if err != nil {
			return Job{}, err
		}
	}
	if req.AppendID != "" {
		source, e := m.Load(owner, req.AppendID)
		if e != nil || source.Action != "generate" || source.ModelID != modelID {
			return Job{}, os.ErrNotExist
		}
		req.AppendFile, err = m.Artifact(owner, req.AppendID, "project.oapjob")
		if err != nil {
			return Job{}, err
		}
	}
	dir := m.dir(owner, id)
	if err = os.MkdirAll(dir, 0700); err != nil {
		return Job{}, err
	}
	defer func() {
		if err != nil {
			os.RemoveAll(dir)
		}
	}()
	req.Model = filepath.Join(m.dir(owner, modelID), "model.step")
	req.Version = 1
	var sourceHash string
	if upload != nil {
		var f *os.File
		f, err = os.OpenFile(req.Model, os.O_CREATE|os.O_EXCL|os.O_WRONLY, 0600)
		if err != nil {
			return Job{}, err
		}
		var size int64
		h := sha256.New()
		size, err = io.Copy(io.MultiWriter(f, h), upload)
		sourceHash = hex.EncodeToString(h.Sum(nil))
		closeErr := f.Close()
		if err == nil {
			err = closeErr
		}
		if err == nil && size == 0 {
			err = errors.New("empty STEP file")
		}
		if err != nil {
			return Job{}, err
		}
	}
	data, err := json.Marshal(req)
	if err != nil {
		return Job{}, err
	}
	if err = os.WriteFile(filepath.Join(dir, "request.json"), data, 0600); err != nil {
		return Job{}, err
	}
	j := Job{ID: id, SourceHash: sourceHash, ModelID: modelID, Action: req.Action, State: "queued", Name: req.SourceName, Created: time.Now().UTC()}
	m.mu.Lock()
	defer m.mu.Unlock()
	if err = save(dir, j); err != nil {
		return Job{}, err
	}
	ctx, cancel := context.WithCancel(context.Background())
	m.active[id] = cancel
	submitted = true
	go m.run(ctx, dir, j)
	return j, nil
}

func (m *Manager) run(ctx context.Context, dir string, j Job) {
	defer func() { <-m.admitted }()
	execute := func() error {
		select {
		case m.slots <- struct{}{}:
			defer func() { <-m.slots }()
		case <-ctx.Done():
			return ctx.Err()
		}
		j.State = "running"
		m.mu.Lock()
		err := save(dir, j)
		m.mu.Unlock()
		if err != nil {
			return err
		}
		limited, cancel := context.WithTimeout(ctx, 30*time.Minute)
		defer cancel()
		cmd := exec.CommandContext(limited, m.worker, "request.json")
		cmd.Dir = dir
		log, err := os.Create(filepath.Join(dir, "worker.log"))
		if err != nil {
			return err
		}
		defer log.Close()
		stdout, err := cmd.StdoutPipe()
		if err != nil {
			return err
		}
		cmd.Stderr = log
		if err = cmd.Start(); err != nil {
			return err
		}
		reader := bufio.NewScanner(stdout)
		for reader.Scan() {
			var event struct{ Stage string }
			if json.Unmarshal(reader.Bytes(), &event) == nil && event.Stage != "" {
				j.Stage = event.Stage
				m.mu.Lock()
				err = save(dir, j)
				m.mu.Unlock()
				if err != nil {
					cancel()
					break
				}
			}
		}
		if reader.Err() != nil {
			cancel()
		}
		if err = cmd.Wait(); err != nil {
			if limited.Err() != nil {
				return limited.Err()
			}
			f, e := os.Open(log.Name())
			if e != nil {
				return err
			}
			defer f.Close()
			if info, e := f.Stat(); e == nil && info.Size() > 1500 {
				_, _ = f.Seek(-1500, io.SeekEnd)
			}
			tail, _ := io.ReadAll(io.LimitReader(f, 1500))
			return fmt.Errorf("worker %w: %s", err, tail)
		}
		data, err := os.ReadFile(filepath.Join(dir, "result.json"))
		if err != nil {
			return err
		}
		var result struct {
			PocketJSONL bool
			Version     int
			Name        string
		}
		if json.Unmarshal(data, &result) != nil || result.Version != 1 {
			return errors.New("invalid worker result")
		}
		if result.Name != "" {
			j.Name = result.Name
		}
		if j.Action == "export" {
			if err := zipPrograms(dir); err != nil {
				return err
			}
		}
		files := outputs(j.Action)
		if j.Action == "rebuild" && result.PocketJSONL {
			files = append(files, "toolpath.jsonl")
		}
		for _, name := range files {
			info, err := os.Stat(filepath.Join(dir, name))
			if err != nil || info.Size() == 0 {
				return fmt.Errorf("missing worker output %s", name)
			}
		}
		j.Files = files
		return nil
	}
	err := execute()
	m.mu.Lock()
	defer m.mu.Unlock()
	j.State = "done"
	if ctx.Err() != nil {
		j.State = "cancelled"
	} else if err != nil {
		j.State = "failed"
		j.Error = err.Error()
	}
	if err := save(dir, j); err != nil {
		fmt.Fprintln(os.Stderr, err)
	}
	m.active[j.ID]()
	delete(m.active, j.ID)
}

func (m *Manager) Cancel(owner, id string) error {
	if _, err := m.Load(owner, id); err != nil {
		return err
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if cancel := m.active[id]; cancel != nil {
		cancel()
	}
	return nil
}

func (m *Manager) Artifact(owner, id, name string) (string, error) {
	j, err := m.Load(owner, id)
	if err != nil || j.State != "done" {
		return "", os.ErrNotExist
	}
	allowed := name == "result.json" || slices.Contains(j.Files, name) || slices.Contains(outputs(j.Action), name)
	if !allowed {
		return "", os.ErrNotExist
	}
	return filepath.Join(m.dir(owner, id), name), nil
}

func outputs(action string) []string {
	switch action {
	case "analyze":
		return []string{"mesh.json", "library.oaptl"}
	case "generate", "rebuild":
		return []string{"path.json", "reference.nc", "project.oapjob"}
	case "export":
		return []string{"programs.json", "programs.zip", "project.oapjob"}
	case "project", "plan", "setups", "holes", "stock":
		return []string{"project.oapjob"}
	case "simulate":
		return []string{"project.oapjob", "stock.json", "differences.json"}
	case "library":
		return []string{"library.oaptl"}
	default:
		return nil
	}
}

func (m *Manager) List(owner string) ([]Job, error) {
	if !ValidID(owner) {
		return nil, os.ErrNotExist
	}
	list := []Job{}
	entries, err := os.ReadDir(filepath.Join(m.root, owner))
	if errors.Is(err, os.ErrNotExist) {
		return list, nil
	}
	if err != nil {
		return nil, err
	}
	for _, entry := range entries {
		if !entry.IsDir() || !ValidID(entry.Name()) {
			continue
		}
		j, err := m.Load(owner, entry.Name())
		if errors.Is(err, os.ErrNotExist) {
			continue
		}
		if err != nil {
			return nil, err
		}
		if j.State == "done" {
			list = append(list, j)
		}
	}
	sort.Slice(list, func(i, k int) bool { return list[i].Created.After(list[k].Created) })
	return list, nil
}
