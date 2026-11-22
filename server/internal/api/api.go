package api

import (
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"strings"

	"openautopath/server/internal/jobs"
)

func respond(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}

func Handler(manager *jobs.Manager, web string) http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET /api/session", func(w http.ResponseWriter, r *http.Request) {
		c, err := r.Cookie("oap_session")
		if err != nil || !jobs.ValidID(c.Value) {
			token, err := jobs.Token()
			if err != nil {
				http.Error(w, "session unavailable", 500)
				return
			}
			http.SetCookie(w, &http.Cookie{Name: "oap_session", Value: token, Path: "/", HttpOnly: true, Secure: r.TLS != nil, SameSite: http.SameSiteStrictMode, MaxAge: 60 * 60 * 24 * 30})
		}
		w.WriteHeader(http.StatusNoContent)
	})
	api := http.NewServeMux()
	owner := func(r *http.Request) string { c, _ := r.Cookie("oap_session"); return c.Value }
	api.HandleFunc("GET /api/workspace", func(w http.ResponseWriter, r *http.Request) {
		items, err := manager.List(owner(r))
		if err != nil {
			http.Error(w, "cannot read workspace", 500)
			return
		}
		respond(w, 200, items)
	})
	documentJob := func(action string) http.HandlerFunc {
		return func(w http.ResponseWriter, r *http.Request) {
			var input struct {
				Document                                    json.RawMessage `json:"document"`
				CompletedMoves                              *int            `json:"completedMoves"`
				ConfirmedSetups                             *[]string       `json:"confirmedSetups"`
				Operation                                   string          `json:"operation"`
				AppendID                                    string          `json:"appendId"`
				Feature, Tool, Strategy, LibraryID, StockID string
				AutoTool                                    bool `json:"autoTool"`
				Step                                        float64
			}
			dec := json.NewDecoder(http.MaxBytesReader(w, r.Body, 32<<20))
			dec.DisallowUnknownFields()
			if err := dec.Decode(&input); err != nil || dec.Decode(new(any)) != io.EOF ||
				(input.CompletedMoves != nil && (action != "simulate" || input.Operation == "" || *input.CompletedMoves < 0)) ||
				(action == "stock" && (input.StockID == "" || len(input.Document) == 0)) ||
				(action == "rebuild" && input.Operation == "") ||
				(action == "setups" && input.ConfirmedSetups == nil) ||
				((action == "simulate" || action == "export" || action == "setups" || action == "rebuild") && len(input.Document) == 0) ||
				(action == "plan" && (input.Feature == "" || (!input.AutoTool && input.Tool == "") || input.Step <= 0 || input.Step > 1000)) {
				http.Error(w, "invalid document request", 400)
				return
			}
			req := jobs.Request{Action: action, CompletedMoves: input.CompletedMoves, StockID: input.StockID, ConfirmedSetups: input.ConfirmedSetups, Document: input.Document, Operation: input.Operation, AppendID: input.AppendID,
				Feature: input.Feature, AutoTool: input.AutoTool, Tool: input.Tool, Strategy: input.Strategy, LibraryID: input.LibraryID, Step: input.Step}
			job, err := manager.Submit(owner(r), r.PathValue("id"), req, nil)
			if errors.Is(err, os.ErrNotExist) {
				http.NotFound(w, r)
				return
			}
			if errors.Is(err, jobs.ErrBusy) {
				http.Error(w, err.Error(), 429)
				return
			}
			if err != nil {
				http.Error(w, "cannot create job", 500)
				return
			}
			respond(w, 202, job)
		}
	}
	api.HandleFunc("POST /api/models/{id}/features", documentJob("features"))
	api.HandleFunc("POST /api/libraries", documentJob("library"))
	api.HandleFunc("POST /api/models/{id}/projects", documentJob("project"))
	api.HandleFunc("POST /api/models/{id}/projects/rebuild", documentJob("rebuild"))
	api.HandleFunc("POST /api/models/{id}/projects/simulate", documentJob("simulate"))
	api.HandleFunc("POST /api/models/{id}/projects/stock", documentJob("stock"))
	api.HandleFunc("POST /api/models/{id}/projects/setups", documentJob("setups"))
	api.HandleFunc("POST /api/models/{id}/projects/holes", documentJob("holes"))
	api.HandleFunc("POST /api/models/{id}/projects/export", documentJob("export"))
	api.HandleFunc("POST /api/models/{id}/projects/plan", documentJob("plan"))
	api.HandleFunc("POST /api/models", func(w http.ResponseWriter, r *http.Request) {
		r.Body = http.MaxBytesReader(w, r.Body, 128<<20)
		name := filepath.Base(strings.ReplaceAll(r.URL.Query().Get("name"), "\\", "/"))
		if name == "." || len(name) > 255 {
			name = "model.step"
		}
		job, err := manager.Submit(owner(r), "", jobs.Request{Action: "analyze", SourceName: name}, r.Body)
		if err != nil {
			var size *http.MaxBytesError
			if errors.Is(err, jobs.ErrBusy) {
				http.Error(w, err.Error(), http.StatusTooManyRequests)
			} else if errors.As(err, &size) {
				http.Error(w, "STEP exceeds 128 MiB", 413)
			} else {
				http.Error(w, err.Error(), 400)
			}
			return
		}
		respond(w, 202, job)
	})
	api.HandleFunc("POST /api/models/{id}/toolpaths", func(w http.ResponseWriter, r *http.Request) {
		var input struct {
			Hole      string
			Tool      string
			LibraryID string
			Feed, RPM float64
		}
		dec := json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096))
		dec.DisallowUnknownFields()
		if dec.Decode(&input) != nil || dec.Decode(new(any)) != io.EOF || input.Hole == "" || input.Tool == "" || input.Feed <= 0 || input.Feed > 100000 || input.RPM <= 0 || input.RPM > 100000 {
			http.Error(w, "invalid machining parameters", 400)
			return
		}
		job, err := manager.Submit(owner(r), r.PathValue("id"), jobs.Request{Action: "generate", Hole: input.Hole, Tool: input.Tool, Feed: input.Feed, RPM: input.RPM, LibraryID: input.LibraryID}, nil)
		if errors.Is(err, os.ErrNotExist) {
			http.NotFound(w, r)
			return
		}
		if errors.Is(err, jobs.ErrBusy) {
			http.Error(w, err.Error(), http.StatusTooManyRequests)
			return
		}
		if err != nil {
			http.Error(w, "cannot create job", 500)
			return
		}
		respond(w, 202, job)
	})
	api.HandleFunc("GET /api/jobs/{id}", func(w http.ResponseWriter, r *http.Request) {
		job, err := manager.Load(owner(r), r.PathValue("id"))
		if err != nil {
			http.NotFound(w, r)
			return
		}
		respond(w, 200, job)
	})
	api.HandleFunc("DELETE /api/jobs/{id}", func(w http.ResponseWriter, r *http.Request) {
		if manager.Cancel(owner(r), r.PathValue("id")) != nil {
			http.NotFound(w, r)
			return
		}
		w.WriteHeader(204)
	})
	api.HandleFunc("GET /api/jobs/{id}/files/{name}", func(w http.ResponseWriter, r *http.Request) {
		name := r.PathValue("name")
		file, err := manager.Artifact(owner(r), r.PathValue("id"), name)
		if err != nil {
			http.NotFound(w, r)
			return
		}
		if name == "programs.zip" || name == "toolpath.jsonl" || name == "reference.nc" || name == "project.oapjob" || name == "library.oaptl" {
			w.Header().Set("Content-Disposition", `attachment; filename="`+name+`"`)
		}
		http.ServeFile(w, r, file)
	})
	mux.Handle("/api/", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		c, err := r.Cookie("oap_session")
		if err != nil || !jobs.ValidID(c.Value) {
			http.Error(w, "session required", 401)
			return
		}
		api.ServeHTTP(w, r)
	}))
	mux.Handle("/", http.FileServer(http.Dir(web)))
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("Cache-Control", "private, no-store")
		mux.ServeHTTP(w, r)
	})
}
