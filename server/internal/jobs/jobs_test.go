package jobs

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func TestCancelQueuedJobAndRestore(t *testing.T) {
	exe, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	root := t.TempDir()
	m, err := New(root, exe, 1)
	if err != nil {
		t.Fatal(err)
	}
	// Occupy the only compute slot so the queued job can be cancelled deterministically.
	m.slots <- struct{}{}
	defer func() { <-m.slots }()
	owner, err := Token()
	if err != nil {
		t.Fatal(err)
	}
	j, err := m.Submit(owner, "", Request{Action: "analyze"}, strings.NewReader("sample"))
	if err != nil {
		t.Fatal(err)
	}
	if err = m.Cancel(owner, j.ID); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		j, err = m.Load(owner, j.ID)
		if err != nil {
			t.Fatal(err)
		}
		if j.State == "cancelled" {
			break
		}
		time.Sleep(5 * time.Millisecond)
	}
	if j.State != "cancelled" {
		t.Fatalf("state %s", j.State)
	}
	if _, err = m.Artifact(owner, j.ID, "result.json"); err == nil {
		t.Fatal("published cancelled output")
	}
	restarted, err := New(root, exe, 1)
	if err != nil {
		t.Fatal(err)
	}
	restored, err := restarted.Load(owner, j.ID)
	if err != nil || restored.State != "cancelled" {
		t.Fatalf("restored %+v %v", restored, err)
	}
	// Capture the on-disk state a server killed mid-job would leave.
	j.State = "running"
	if err = save(filepath.Join(root, owner, j.ID), j); err != nil {
		t.Fatal(err)
	}
	restored, err = restarted.Load(owner, j.ID)
	if err != nil || restored.State != "failed" {
		t.Fatalf("interrupted %+v %v", restored, err)
	}
}

func TestRejectedNCArtifacts(t *testing.T) {
	root := t.TempDir()
	owner, _ := Token()
	id, _ := Token()
	exe, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	m, err := New(root, exe, 1)
	if err != nil {
		t.Fatal(err)
	}
	dir := filepath.Join(root, owner, id)
	if err := os.MkdirAll(dir, 0700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "programs.json"), []byte(`[{"filename":"001-G54-T1.nc","text":"example","errors":[{"line":1}]}]`), 0600); err != nil {
		t.Fatal(err)
	}
	if err := zipPrograms(dir); err == nil {
		t.Fatal("archived NC with structural errors")
	}
	for _, action := range []string{"export", "rebuild"} {
		if err := save(dir, Job{ID: id, Action: action, State: "done", Files: []string{"programs.json", "project.oapjob"}}); err != nil {
			t.Fatal(err)
		}
		for _, name := range []string{"reference.nc", "programs.zip"} {
			if _, err := m.Artifact(owner, id, name); !os.IsNotExist(err) {
				t.Fatalf("unexpected artifact %s: %v", name, err)
			}
		}
		if _, err := m.Artifact(owner, id, "programs.json"); err != nil {
			t.Fatal(err)
		}
	}
}
