// Package jobs runs each immutable model request in a separate working directory.
package jobs

import (
	"bufio"
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"sync"
	"time"
)

type Request struct {
	Version int     `json:"version"`
	Action  string  `json:"action"`
	Model   string  `json:"model"`
	Hole    string  `json:"hole,omitempty"`
	Tool    string  `json:"tool,omitempty"`
	Feed    float64 `json:"feed,omitempty"`
	RPM     float64 `json:"rpm,omitempty"`
}

type Job struct {
	Stage   string `json:"stage,omitempty"`
	ID      string `json:"id"`
	ModelID string `json:"modelId"`
	Action  string `json:"action"`
	State   string `json:"state"`
	Error   string `json:"error,omitempty"`
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
	} else {
		model, err := m.Load(owner, modelID)
		if err != nil || model.State != "done" || model.Action != "analyze" {
			return Job{}, os.ErrNotExist
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
	if upload != nil {
		var f *os.File
		f, err = os.OpenFile(req.Model, os.O_CREATE|os.O_EXCL|os.O_WRONLY, 0600)
		if err != nil {
			return Job{}, err
		}
		var size int64
		size, err = io.Copy(f, upload)
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
	j := Job{ID: id, ModelID: modelID, Action: req.Action, State: "queued"}
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
		var result struct{ Version int }
		if json.Unmarshal(data, &result) != nil || result.Version != 1 {
			return errors.New("invalid worker result")
		}
		files := []string{"mesh.json"}
		if j.Action == "generate" {
			files = []string{"path.json", "reference.nc"}
		}
		for _, name := range files {
			info, err := os.Stat(filepath.Join(dir, name))
			if err != nil || info.Size() == 0 {
				return fmt.Errorf("missing worker output %s", name)
			}
		}
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
	allowed := name == "result.json" || (j.Action == "analyze" && name == "mesh.json") || (j.Action == "generate" && (name == "path.json" || name == "reference.nc"))
	if !allowed {
		return "", os.ErrNotExist
	}
	return filepath.Join(m.dir(owner, id), name), nil
}
