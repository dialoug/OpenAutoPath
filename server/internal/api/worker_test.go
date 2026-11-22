package api

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"net/http/cookiejar"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"openautopath/server/internal/jobs"
)

// Run with OAP_TEST_WORKER pointing to a native worker and its runtime on PATH.
func TestNativeCAMWorkflow(t *testing.T) {
	worker := os.Getenv("OAP_TEST_WORKER")
	if worker == "" {
		t.Skip("set OAP_TEST_WORKER for native integration")
	}
	manager, err := jobs.New(t.TempDir(), worker, 1)
	if err != nil {
		t.Fatal(err)
	}
	host := httptest.NewServer(Handler(manager, t.TempDir()))
	defer host.Close()
	jar, _ := cookiejar.New(nil)
	client := &http.Client{Jar: jar, Timeout: time.Minute}
	call := func(method, path string, body io.Reader, code int) []byte {
		t.Helper()
		req, err := http.NewRequest(method, host.URL+path, body)
		if err != nil {
			t.Fatal(err)
		}
		resp, err := client.Do(req)
		if err != nil {
			t.Fatal(err)
		}
		defer resp.Body.Close()
		data, err := io.ReadAll(resp.Body)
		if err != nil {
			t.Fatal(err)
		}
		if resp.StatusCode != code {
			t.Fatalf("%s %s: %d %s", method, path, resp.StatusCode, data)
		}
		return data
	}
	wait := func(data []byte, want string) jobs.Job {
		t.Helper()
		var j jobs.Job
		if err := json.Unmarshal(data, &j); err != nil {
			t.Fatal(err)
		}
		end := time.Now().Add(2 * time.Minute)
		for (j.State == "queued" || j.State == "running") && time.Now().Before(end) {
			time.Sleep(100 * time.Millisecond)
			if err := json.Unmarshal(call("GET", "/api/jobs/"+j.ID, nil, 200), &j); err != nil {
				t.Fatal(err)
			}
		}
		if j.State != want {
			t.Fatalf("job %s: %s %s", j.ID, j.State, j.Error)
		}
		return j
	}
	call("GET", "/api/session", nil, 204)
	invalid := wait(call("POST", "/api/models", strings.NewReader("broken STEP"), 202), "failed")
	call("GET", "/api/jobs/"+invalid.ID+"/files/result.json", nil, 404)
	sample, err := os.Open(filepath.Join("..", "..", "..", "samples", "cam_feature_test.stp"))
	if err != nil {
		t.Fatal(err)
	}
	defer sample.Close()
	model := wait(call("POST", "/api/models", sample, 202), "done")
	base := "/api/jobs/" + model.ID + "/files/"
	var result struct {
		Features []struct{ ID, Kind string }
		Holes    []struct {
			ID       string
			Diameter float64
			Faces    []string
		}
		Tools []struct {
			ID, Type string
			Diameter float64
		}
	}
	if err := json.Unmarshal(call("GET", base+"result.json", nil, 200), &result); err != nil {
		t.Fatal(err)
	}
	var mesh []struct {
		ID      string
		Indices []int
	}
	if err := json.Unmarshal(call("GET", base+"mesh.json", nil, 200), &mesh); err != nil {
		t.Fatal(err)
	}
	if len(mesh) != 34 || len(result.Holes) != 8 {
		t.Fatalf("sample geometry changed: %d faces, %d holes", len(mesh), len(result.Holes))
	}
	faceIDs := map[string]bool{}
	for _, f := range mesh {
		faceIDs[f.ID] = len(f.Indices) > 0
	}
	var holeID, toolID string
	for _, hole := range result.Holes {
		for _, face := range hole.Faces {
			if !faceIDs[face] {
				t.Fatalf("unpickable face %s", face)
			}
		}
		if math.Abs(hole.Diameter-6) < .001 {
			holeID = hole.ID
			for _, tool := range result.Tools {
				if tool.Type == "drill" && math.Abs(tool.Diameter-6) < .001 {
					toolID = tool.ID
					break
				}
			}
			break
		}
	}
	if holeID == "" || toolID == "" {
		t.Fatal("missing sample 6 mm hole or matching drill")
	}
	inputs := fmt.Sprintf(`{"hole":%q,"tool":%q,"feed":100,"rpm":3000}`, holeID, toolID)
	pathJob := wait(call("POST", "/api/models/"+model.ID+"/toolpaths", strings.NewReader(inputs), 202), "done")
	base = "/api/jobs/" + pathJob.ID + "/files/"
	nc := call("GET", base+"reference.nc", nil, 200)
	for _, token := range []string{"OpenAutoPath hole", "REFERENCE NC", "S3000", "F100.", "M30"} {
		if !bytes.Contains(nc, []byte(token)) {
			t.Fatalf("missing %q in reference NC", token)
		}
	}
	var path [][][]float64
	if json.Unmarshal(call("GET", base+"path.json", nil, 200), &path) != nil || len(path) == 0 {
		t.Fatal("missing toolpath preview")
	}
	t.Logf("%d holes, %d mesh faces, %d moves, %d NC bytes", len(result.Holes), len(mesh), len(path), len(nc))
	docRequest := func(doc []byte, other map[string]any) io.Reader {
		if other == nil {
			other = map[string]any{}
		}
		if doc != nil {
			other["document"] = json.RawMessage(doc)
		}
		payload, e := json.Marshal(other)
		if e != nil {
			t.Fatal(e)
		}
		return bytes.NewReader(payload)
	}
	lib := call("GET", "/api/jobs/"+model.ID+"/files/library.oaptl", nil, 200)
	custom := bytes.ReplaceAll(lib, []byte(toolID), []byte("web-test-drill"))
	libraryJob := wait(call("POST", "/api/libraries", docRequest(custom, nil), 202), "done")
	changedInput := fmt.Sprintf(`{"hole":%q,"tool":"web-test-drill","feed":88,"rpm":2500,"libraryId":%q}`, holeID, libraryJob.ID)
	generated := wait(call("POST", "/api/models/"+model.ID+"/toolpaths", strings.NewReader(changedInput), 202), "done")
	projectURL := "/api/models/" + model.ID + "/projects"
	blank := wait(call("POST", projectURL, strings.NewReader("{}"), 202), "done")
	readProject := func(j jobs.Job) []byte { return call("GET", "/api/jobs/"+j.ID+"/files/project.oapjob", nil, 200) }
	joined := wait(call("POST", projectURL, docRequest(readProject(blank), map[string]any{"appendId": generated.ID}), 202), "done")
	project := readProject(joined)
	var saved struct {
		Job struct {
			HoleOperations []struct{ ID string } `json:"hole_operations"`
		}
	}
	if json.Unmarshal(project, &saved) != nil || len(saved.Job.HoleOperations) != 1 {
		t.Fatal("missing captured operation")
	}
	id := saved.Job.HoleOperations[0].ID
	rebuild := wait(call("POST", projectURL+"/rebuild", docRequest(project, map[string]any{"operation": id}), 202), "done")
	nc = call("GET", "/api/jobs/"+rebuild.ID+"/files/reference.nc", nil, 200)
	if !bytes.Contains(nc, []byte("F88.")) || !bytes.Contains(nc, []byte("S2500")) {
		t.Fatal("project lost cutter snapshot")
	}
	var edited map[string]any
	if err := json.Unmarshal(project, &edited); err != nil {
		t.Fatal(err)
	}
	def := edited["job"].(map[string]any)
	snapshot := def["tool_controllers"].([]any)[0].(map[string]any)["tool"].(map[string]any)
	snapshot["vertical_feed"], snapshot["spindle_speed"] = 77., 2700.
	updated, err := json.Marshal(edited)
	if err != nil {
		t.Fatal(err)
	}
	revised := wait(call("POST", projectURL+"/rebuild", docRequest(updated, map[string]any{"operation": id}), 202), "done")
	revisedNC := call("GET", "/api/jobs/"+revised.ID+"/files/reference.nc", nil, 200)
	if !bytes.Contains(revisedNC, []byte("F77.")) || !bytes.Contains(revisedNC, []byte("S2700")) {
		t.Fatal("edits to project cutter did not reach NC")
	}
	def["collision_scene"] = map[string]any{"confirmed": false, "models": []any{map[string]any{"source_path": "fixture.step", "source_fingerprint": "fixture", "role": 1}}}
	external, err := json.Marshal(edited)
	if err != nil {
		t.Fatal(err)
	}
	fixture := wait(call("POST", projectURL, docRequest(external, nil), 202), "done")
	if !bytes.Contains(readProject(fixture), []byte("fixture.step")) {
		t.Fatal("external fixture reference lost on save")
	}
	wait(call("POST", projectURL+"/simulate", docRequest(readProject(fixture), nil), 202), "failed")
	simulated := wait(call("POST", projectURL+"/simulate", docRequest(project, nil), 202), "done")
	var stock struct {
		Volume     float64
		Operations int
	}
	if json.Unmarshal(call("GET", "/api/jobs/"+simulated.ID+"/files/result.json", nil, 200), &stock) != nil || stock.Volume <= 0 || stock.Operations != 1 {
		t.Fatalf("incorrect IPW result: %+v", stock)
	}
	var stockMesh []struct{ Indices []int }
	if json.Unmarshal(call("GET", "/api/jobs/"+simulated.ID+"/files/stock.json", nil, 200), &stockMesh) != nil || len(stockMesh) != 1 || len(stockMesh[0].Indices) < 3 {
		t.Fatal("missing IPW triangles")
	}

	millingTool := ""
	for _, tool := range result.Tools {
		if tool.Type == "end_mill" && tool.Diameter == 6 {
			millingTool = tool.ID
			break
		}
	}
	if millingTool == "" {
		t.Fatal("missing end mill")
	}
	plannedKinds := map[string]bool{}
	for _, feature := range result.Features {
		if plannedKinds[feature.Kind] {
			continue
		}
		strategy := map[string]string{"pocket": "offset", "profile": "contour", "engrave": "finish"}[feature.Kind]
		j := wait(call("POST", projectURL+"/plan", docRequest(project, map[string]any{"feature": feature.ID, "tool": millingTool, "strategy": strategy, "step": 2}), 202), "done")
		var planned struct{ Job map[string]json.RawMessage }
		project = readProject(j)
		if json.Unmarshal(project, &planned) != nil {
			t.Fatal("unreadable planned project")
		}
		field := map[string]string{"pocket": "pocket_operations", "profile": "profile_operations", "engrave": "surface_finishing_operations"}[feature.Kind]
		var ops []struct{ ID string }
		if json.Unmarshal(planned.Job[field], &ops) != nil || len(ops) == 0 {
			t.Fatalf("missing %s operations", feature.Kind)
		}
		rebuilt := wait(call("POST", projectURL+"/rebuild", docRequest(readProject(j), map[string]any{"operation": ops[0].ID}), 202), "done")
		if len(call("GET", "/api/jobs/"+rebuilt.ID+"/files/path.json", nil, 200)) < 10 {
			t.Fatal("empty planned toolpath")
		}
		plannedKinds[feature.Kind] = true
	}
	if len(plannedKinds) != 3 {
		t.Fatalf("missing milling families: %v", plannedKinds)
	}
	mixed := wait(call("POST", projectURL+"/simulate", docRequest(project, nil), 202), "done")
	if json.Unmarshal(call("GET", "/api/jobs/"+mixed.ID+"/files/result.json", nil, 200), &stock) != nil || stock.Operations < 4 {
		t.Fatalf("missing mixed IPW operations: %+v", stock)
	}
	imported := wait(call("POST", projectURL, docRequest(project, nil), 202), "done")
	if !bytes.Contains(readProject(imported), []byte(id)) {
		t.Fatal("operation lost during import")
	}
	wait(call("POST", "/api/libraries", strings.NewReader(`{"document":{"schema":"invalid"}}`), 202), "failed")
	client.Jar, _ = cookiejar.New(nil)
	call("GET", "/api/session", nil, 204)
	call("GET", base+"reference.nc", nil, 404)
	call("GET", "/api/jobs/"+joined.ID+"/files/project.oapjob", nil, 404)
	if data := call("GET", "/api/workspace", nil, 200); strings.TrimSpace(string(data)) != "[]" {
		t.Fatal("workspace leaked other session")
	}
	call("POST", "/api/models/"+model.ID+"/toolpaths", strings.NewReader(inputs), 404)
}
