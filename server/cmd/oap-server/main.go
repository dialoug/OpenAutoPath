package main

import (
	"flag"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"runtime"
	"time"

	"openautopath/server/internal/api"
	"openautopath/server/internal/jobs"
)

func main() {
	executable, err := os.Executable()
	if err != nil {
		log.Fatal(err)
	}
	packageDir := filepath.Dir(executable)
	workerName := "oap-worker"
	if runtime.GOOS == "windows" {
		workerName += ".exe"
	}
	addr := flag.String("listen", "127.0.0.1:8080", "HTTP listen address")
	worker := flag.String("worker", filepath.Join(packageDir, "bin", workerName), "worker executable path")
	data := flag.String("data", "data", "persistent jobs directory")
	web := flag.String("web", filepath.Join(packageDir, "web"), "built web assets directory")
	limit := flag.Int("workers", 1, "concurrent CAM processes")
	flag.Parse()
	if _, err := os.Stat(filepath.Join(*web, "index.html")); err != nil {
		log.Fatal(err)
	}
	m, err := jobs.New(*data, *worker, *limit)
	if err != nil {
		log.Fatal(err)
	}
	server := http.Server{
		Addr: *addr, Handler: api.Handler(m, *web),
		ReadHeaderTimeout: 10 * time.Second, ReadTimeout: 10 * time.Minute,
		IdleTimeout: 60 * time.Second,
	}
	log.Printf("OpenAutoPath listening on %s", *addr)
	log.Fatal(server.ListenAndServe())
}
