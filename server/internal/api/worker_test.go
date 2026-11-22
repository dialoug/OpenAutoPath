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
		Holes []struct {
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
	client.Jar, _ = cookiejar.New(nil)
	call("GET", "/api/session", nil, 204)
	call("GET", base+"reference.nc", nil, 404)
	call("POST", "/api/models/"+model.ID+"/toolpaths", strings.NewReader(inputs), 404)
}
