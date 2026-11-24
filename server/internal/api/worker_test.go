package api

import (
	"archive/zip"
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
	"slices"
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
	root := t.TempDir()
	manager, err := jobs.New(root, worker, 1)
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
	cacheFiles, err := filepath.Glob(filepath.Join(root, "*", model.ID, "model.step.bounds"))
	if err != nil || len(cacheFiles) != 1 {
		t.Fatalf("expected private face bounds for uploaded model: %v, %v", cacheFiles, err)
	}
	cachePath := cacheFiles[0]
	originalBounds, err := os.ReadFile(cachePath)
	if err != nil || len(originalBounds) <= 64 {
		t.Fatalf("missing cached bounds: %v", err)
	}
	// A warm worker must use the existing entry, even across independent jobs.
	stamp := time.Unix(1000000000, 0)
	if err := os.Chtimes(cachePath, stamp, stamp); err != nil {
		t.Fatal(err)
	}
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
	backfill := wait(call("POST", "/api/models/"+model.ID+"/features", strings.NewReader("{}"), 202), "done")
	var regions struct{ Features []struct{ ID, Kind string } }
	if err := json.Unmarshal(call("GET", "/api/jobs/"+backfill.ID+"/files/result.json", nil, 200), &regions); err != nil || !slices.Equal(regions.Features, result.Features) || backfill.ModelID != model.ID {
		t.Fatal("backfilled features differ from initial analysis")
	}
	if stat, err := os.Stat(cachePath); err != nil || !stat.ModTime().Equal(stamp) {
		t.Fatal("second worker rebuilt the face bounds")
	}
	call("GET", base+"model.step.bounds", nil, 404)
	damaged := bytes.Clone(originalBounds)
	damaged[len(damaged)-1] ^= 1
	if err := os.WriteFile(cachePath, damaged, 0600); err != nil {
		t.Fatal(err)
	}
	wait(call("POST", "/api/models/"+model.ID+"/features", strings.NewReader("{}"), 202), "done")
	if repaired, err := os.ReadFile(cachePath); err != nil || !bytes.Equal(repaired, originalBounds) {
		t.Fatal("worker did not recover the original bounds from a corrupt cache")
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
	var single []struct {
		Filename, Text string
		Warnings       []struct {
			Line int
			Code string
		}
	}
	if json.Unmarshal(call("GET", base+"programs.json", nil, 200), &single) != nil || len(single) != 1 || single[0].Text != string(nc) || single[0].Filename != "reference.nc" {
		t.Fatal("single NC review differs from downloadable program")
	}
	sourceLines := strings.Split(string(nc), "\n")
	foundReference := false
	for _, warning := range single[0].Warnings {
		if warning.Code == "reference_nc_only" && warning.Line > 0 && warning.Line <= len(sourceLines) {
			foundReference = strings.Contains(sourceLines[warning.Line-1], "REFERENCE NC")
		}
	}
	if !foundReference {
		t.Fatal("reference warning has no matching source line")
	}
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
	var diff struct {
		Comparison struct{ Allowance, Overcut, Missing, CertainOvercut float64 }
	}
	if err := json.Unmarshal(call("GET", "/api/jobs/"+simulated.ID+"/files/result.json", nil, 200), &diff); err != nil || diff.Comparison.Allowance <= 0 || diff.Comparison.CertainOvercut > diff.Comparison.Overcut {
		t.Fatalf("invalid material comparison: %+v", diff)
	}
	var views map[string]json.RawMessage
	if json.Unmarshal(call("GET", "/api/jobs/"+simulated.ID+"/files/differences.json", nil, 200), &views) != nil || len(views["allowance"]) == 0 {
		t.Fatal("missing allowance geometry")
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
	// Exercise two physical pockets in the same saved project.
	{
		type pocket struct {
			ID         string
			Sequence   int
			Controller string `json:"tool_controller_id"`
			Target     struct {
				Wire string `json:"root_wire_id"`
			}
			Parameters struct {
				Depth struct {
					Step float64 `json:"step_down"`
				}
				Clearing struct{ Pattern int }
			}
		}
		type controller struct {
			ID   string
			Tool struct {
				ID string `json:"tool_id"`
			}
		}
		read := func(j jobs.Job) ([]pocket, map[string]string) {
			var doc struct {
				Job struct {
					Pockets     []pocket     `json:"pocket_operations"`
					Controllers []controller `json:"tool_controllers"`
				}
			}
			if err := json.Unmarshal(readProject(j), &doc); err != nil {
				t.Fatal(err)
			}
			ids := map[string]string{}
			for _, c := range doc.Job.Controllers {
				ids[c.ID] = c.Tool.ID
			}
			return doc.Job.Pockets, ids
		}
		var pockets []string
		for _, f := range result.Features {
			if f.Kind == "pocket" {
				pockets = append(pockets, f.ID)
			}
		}
		if len(pockets) < 2 {
			t.Fatal("need two distinct pockets")
		}
		plan := func(j jobs.Job, feature, tool, strategy string, step float64) jobs.Job {
			return wait(call("POST", projectURL+"/plan", docRequest(readProject(j), map[string]any{"feature": feature, "tool": tool, "step": step, "strategy": strategy}), 202), "done")
		}
		calc := func(j jobs.Job, id, tool string) jobs.Job {
			return wait(call("POST", projectURL+"/rebuild", docRequest(readProject(j), map[string]any{"operation": id, "tool": tool}), 202), "done")
		}
		ncFor := func(j jobs.Job) []byte { return call("GET", "/api/jobs/"+j.ID+"/files/reference.nc", nil, 200) }
		one := wait(call("POST", projectURL+"/plan", docRequest(nil, map[string]any{"feature": pockets[0], "tool": millingTool, "strategy": "offset", "step": 2}), 202), "done")
		ops, _ := read(one)
		if len(ops) != 1 {
			t.Fatal("first pocket missing")
		}
		first := ops[0]
		firstBefore := calc(one, first.ID, "")
		pathFor := func(j jobs.Job) []byte { return call("GET", "/api/jobs/"+j.ID+"/files/path.json", nil, 200) }
		two := plan(one, pockets[1], "sim-v1-end_mill-d4", "zigzag", 1)
		ops, tools := read(two)
		if len(ops) != 2 || first.Parameters.Depth.Step != 2 || first.Parameters.Clearing.Pattern != 0 || ops[1].Parameters.Clearing.Pattern != 1 || ops[0] != first || ops[1].Target.Wire == first.Target.Wire || ops[1].Parameters.Depth.Step != 1 || tools[ops[1].Controller] != "sim-v1-end_mill-d4" {
			t.Fatalf("unexpected pocket pair: %+v", ops)
		}
		before := calc(two, ops[1].ID, "")
		switched := calc(two, ops[1].ID, "sim-v1-end_mill-d3")
		after, tools := read(switched)
		if len(after) != 2 || after[0] != first || after[1].ID != ops[1].ID || after[1].Parameters != ops[1].Parameters || tools[after[1].Controller] != "sim-v1-end_mill-d3" {
			t.Fatal("switching cutter changed unrelated inputs")
		}
		firstAfter := calc(switched, first.ID, "")
		if bytes.Equal(pathFor(before), pathFor(switched)) || !bytes.Equal(pathFor(firstBefore), pathFor(firstAfter)) || bytes.Equal(ncFor(before), ncFor(switched)) || !bytes.Equal(ncFor(firstBefore), ncFor(firstAfter)) {
			t.Fatal("unexpected NC after changing the second cutter")
		}
		stockJob := wait(call("POST", projectURL+"/simulate", docRequest(readProject(switched), nil), 202), "done")
		if json.Unmarshal(call("GET", "/api/jobs/"+stockJob.ID+"/files/result.json", nil, 200), &stock) != nil || stock.Operations != 2 || stock.Volume <= 0 {
			t.Fatal("dual pocket simulation incomplete")
		}
		wait(call("POST", projectURL+"/rebuild", docRequest(readProject(switched), map[string]any{"operation": after[1].ID, "tool": toolID}), 202), "failed")
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

	// The playback mesh stream and the exact inspection endpoint must describe
	// the same material at a sampled move, including preceding operations.
	materialReplay := wait(call("POST", projectURL+"/replay", docRequest(project, nil), 202), "done")
	replayBase := "/api/jobs/" + materialReplay.ID + "/files/"
	var timeline struct {
		Quality    int
		Moves      int
		Operations []struct {
			ID  string
			End int
		}
		Frames []struct {
			Move, Vertices, Triangles               int
			Volume, MaterialSpacing, DisplaySpacing float64
		}
	}
	if json.Unmarshal(call("GET", replayBase+"result.json", nil, 200), &timeline) != nil ||
		timeline.Quality != 5 || len(timeline.Operations) != stock.Operations || len(timeline.Frames) < 3 || len(timeline.Frames) > 33 {
		t.Fatal("invalid material playback manifest")
	}
	first, last := timeline.Frames[0], timeline.Frames[len(timeline.Frames)-1]
	if first.Move != 0 || last.Move != timeline.Moves || math.Abs(last.Volume-stock.Volume) > 1e-5 {
		t.Fatal("playback does not end at the project stock checkpoint")
	}
	size := 0
	for i, f := range timeline.Frames {
		if f.Vertices < 1 || f.Triangles < 1 || f.MaterialSpacing <= 0 || f.DisplaySpacing < f.MaterialSpacing || (i > 0 && f.Move <= timeline.Frames[i-1].Move) {
			t.Fatal("invalid playback frame")
		}
		size += (f.Vertices + f.Triangles) * 12
	}
	if len(call("GET", replayBase+"replay.bin", nil, 200)) != size {
		t.Fatal("playback mesh stream length mismatch")
	}
	middle := timeline.Frames[len(timeline.Frames)/2]
	call("POST", projectURL+"/replay", docRequest(project, map[string]any{"quality": 11}), 400)
	detailed := wait(call("POST", projectURL+"/replay", docRequest(project, map[string]any{"quality": 10, "operation": timeline.Operations[0].ID}), 202), "done")
	var fine struct {
		Quality int
		Frames  []struct{ MaterialSpacing, DisplaySpacing float64 }
	}
	if json.Unmarshal(call("GET", "/api/jobs/"+detailed.ID+"/files/result.json", nil, 200), &fine) != nil || fine.Quality != 10 || len(fine.Frames) < 2 || fine.Frames[0].MaterialSpacing >= first.MaterialSpacing || fine.Frames[0].DisplaySpacing > first.DisplaySpacing {
		t.Fatal("detailed replay failed to refine the sample stock")
	}
	start := 0
	for _, op := range timeline.Operations {
		if middle.Move <= op.End {
			exact := wait(call("POST", projectURL+"/simulate", docRequest(project, map[string]any{"operation": op.ID, "completedMoves": middle.Move - start}), 202), "done")
			var sample struct{ Volume float64 }
			if json.Unmarshal(call("GET", "/api/jobs/"+exact.ID+"/files/result.json", nil, 200), &sample) != nil || math.Abs(sample.Volume-middle.Volume) > 1e-5 {
				t.Fatalf("sampled volume %.9f differs from exact volume %.9f", middle.Volume, sample.Volume)
			}
			break
		}
		start = op.End
	}

	prefix := wait(call("POST", projectURL+"/simulate", docRequest(project, map[string]any{"operation": id}), 202), "done")
	if json.Unmarshal(call("GET", "/api/jobs/"+prefix.ID+"/files/result.json", nil, 200), &stock) != nil || stock.Operations != 1 {
		t.Fatalf("invalid IPW prefix: %+v", stock)
	}
	if !bytes.Contains(readProject(prefix), []byte("surface_finishing_operations")) {
		t.Fatal("partial simulation changed project")
	}
	rejected := wait(call("POST", projectURL+"/export", docRequest(project, nil), 202), "failed")
	call("GET", "/api/jobs/"+rejected.ID+"/files/programs.zip", nil, 404)
	var setupDoc struct {
		Job struct{ Setups []struct{ ID string } }
	}
	if err := json.Unmarshal(project, &setupDoc); err != nil {
		t.Fatal(err)
	}
	var setupIDs []string
	for _, s := range setupDoc.Job.Setups {
		setupIDs = append(setupIDs, s.ID)
	}
	confirmed := wait(call("POST", projectURL+"/setups", docRequest(project, map[string]any{"confirmedSetups": setupIDs}), 202), "done")
	exported := wait(call("POST", projectURL+"/export", docRequest(readProject(confirmed), nil), 202), "done")
	packBase := "/api/jobs/" + exported.ID + "/files/"
	var reviewed []struct {
		Filename, Text string
		Operations     []string
	}
	if json.Unmarshal(call("GET", packBase+"programs.json", nil, 200), &reviewed) != nil || len(reviewed) < 2 {
		t.Fatal("missing reviewed program groups")
	}
	zipped := call("GET", packBase+"programs.zip", nil, 200)
	archive, e := zip.NewReader(bytes.NewReader(zipped), int64(len(zipped)))
	if e != nil || len(archive.File) != len(reviewed) {
		t.Fatal("invalid program archive", e)
	}
	for i, member := range archive.File {
		r, e := member.Open()
		if e != nil {
			t.Fatal(e)
		}
		raw, e := io.ReadAll(r)
		r.Close()
		if e != nil || member.Name != reviewed[i].Filename || string(raw) != reviewed[i].Text || len(reviewed[i].Operations) == 0 {
			t.Fatal("review differs from export")
		}
	}
	batch := wait(call("POST", projectURL+"/holes", docRequest(project, nil), 202), "done")
	var batched struct{ Job map[string]json.RawMessage }
	if json.Unmarshal(readProject(batch), &batched) != nil {
		t.Fatal("invalid batch project")
	}
	for _, key := range []string{"hole_operations", "pocket_operations", "profile_operations", "surface_finishing_operations"} {
		var ops []any
		if json.Unmarshal(batched.Job[key], &ops) != nil || len(ops) == 0 {
			t.Fatal("batch lost " + key)
		}
	}
	for _, moves := range []int{0, 7} {
		frame := wait(call("POST", projectURL+"/simulate", docRequest(project, map[string]any{"operation": id, "completedMoves": moves}), 202), "done")
		var progress struct {
			CompletedMoves, TotalMoves int
			Volume, RemovedVolume      float64
		}
		if json.Unmarshal(call("GET", "/api/jobs/"+frame.ID+"/files/result.json", nil, 200), &progress) != nil || progress.CompletedMoves != moves || progress.TotalMoves < 14 || progress.Volume <= 0 || (moves == 0 && progress.RemovedVolume != 0) {
			t.Fatalf("incorrect IPW frame %+v", progress)
		}
	}
	stockBound := wait(call("POST", projectURL+"/stock", docRequest(project, map[string]any{"stockId": model.ID}), 202), "done")
	boundDoc := readProject(stockBound)
	var bound struct {
		Job struct {
			Stock struct {
				Hash string `json:"source_fingerprint"`
				Kind int    `json:"source_kind"`
				Path string `json:"source_path"`
			}
		}
	}
	if json.Unmarshal(boundDoc, &bound) != nil || bound.Job.Stock.Kind != 2 || len(bound.Job.Stock.Hash) != 64 || filepath.IsAbs(bound.Job.Stock.Path) {
		t.Fatal("invalid portable stock binding")
	}
	wait(call("POST", projectURL+"/simulate", docRequest(boundDoc, map[string]any{"operation": id}), 202), "done")
	var missingStock map[string]any
	if err := json.Unmarshal(boundDoc, &missingStock); err != nil {
		t.Fatal(err)
	}
	missingStock["job"].(map[string]any)["stock"].(map[string]any)["source_fingerprint"] = strings.Repeat("0", 64)
	changed, err := json.Marshal(missingStock)
	if err != nil {
		t.Fatal(err)
	}
	wait(call("POST", projectURL+"/simulate", docRequest(changed, nil), 202), "failed")
	for _, group := range []struct {
		kind, field, strategy string
		min                   int
		auto                  bool
	}{
		{"pocket-all", "pocket_operations", "offset", 2, true}, {"profile-all", "profile_operations", "contour", 6, false},
	} {
		args := map[string]any{"feature": group.kind, "tool": millingTool, "autoTool": group.auto, "strategy": group.strategy, "step": 2}
		if group.kind == "profile-all" {
			rejected := wait(call("POST", projectURL+"/plan", docRequest(project, args), 202), "failed")
			if !strings.Contains(rejected.Error, "tool reach") {
				t.Fatal("expected native reach validation", rejected.Error)
			}
			// The side setups of this fixture span 120 mm. Use a distinct long-reach test library.
			var source map[string]any
			if e := json.Unmarshal(lib, &source); e != nil {
				t.Fatal(e)
			}
			for _, item := range source["library"].(map[string]any)["tools"].([]any) {
				bit := item.(map[string]any)
				if bit["id"] == millingTool {
					g := bit["geometry"].(map[string]any)
					g["overall_length"] = 160
					g["cutting_edge_length"] = 130
				}
			}
			raw, e := json.Marshal(source)
			if e != nil {
				t.Fatal(e)
			}
			args["libraryId"] = wait(call("POST", "/api/libraries", docRequest(raw, nil), 202), "done").ID
		}
		all := wait(call("POST", projectURL+"/plan", docRequest(project, args), 202), "done")
		var data struct{ Job map[string]json.RawMessage }
		var regions []struct {
			Target struct {
				Root string `json:"root_wire_id"`
			}
		}
		if json.Unmarshal(readProject(all), &data) != nil || json.Unmarshal(data.Job[group.field], &regions) != nil {
			t.Fatal("invalid batch plan")
		}
		roots := map[string]bool{}
		for _, r := range regions {
			roots[r.Target.Root] = true
		}
		if len(roots) < group.min {
			t.Fatalf("missing %s references: %v", group.kind, roots)
		}
	}
	six := wait(call("POST", projectURL+"/plan", docRequest(project, map[string]any{"feature": "engrave-all", "strategy": "finish", "step": 2, "tool": millingTool}), 202), "done")
	var sixDoc struct {
		Job struct {
			Ops []struct{ ID string } `json:"surface_finishing_operations"`
		}
	}
	if json.Unmarshal(readProject(six), &sixDoc) != nil || len(sixDoc.Job.Ops) != 6 {
		t.Fatal("incomplete six face job")
	}
	// The fixture's rounded rectangular pocket has its planar floor at Z=12 (F32).
	floor := "F32"
	manual := wait(call("POST", projectURL+"/plan", docRequest(project, map[string]any{"feature": "manual-pocket-" + floor, "strategy": "offset", "step": 2, "tool": millingTool}), 202), "done")
	var manualDoc struct {
		Job struct {
			Ops []struct{ ID string } `json:"pocket_operations"`
		}
	}
	if json.Unmarshal(readProject(manual), &manualDoc) != nil || len(manualDoc.Job.Ops) < 2 {
		t.Fatal("manual floor lost")
	}
	replay := wait(call("POST", projectURL+"/rebuild", docRequest(readProject(manual), map[string]any{"operation": manualDoc.Job.Ops[len(manualDoc.Job.Ops)-1].ID}), 202), "done")
	if len(call("GET", "/api/jobs/"+replay.ID+"/files/toolpath.jsonl", nil, 200)) < 10 {
		t.Fatal("empty pocket JSONL")
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
