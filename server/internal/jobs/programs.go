package jobs

import (
	"archive/zip"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"regexp"
)

var programName = regexp.MustCompile(`^[0-9]+-G5[4-9]-T[0-9]+\.nc$`)

// Each ZIP member contains the exact text shown in the NC review.
func zipPrograms(dir string) error {
	raw, err := os.ReadFile(filepath.Join(dir, "programs.json"))
	if err != nil {
		return err
	}
	var programs []struct {
		Filename, Text string
		Errors         []json.RawMessage
	}
	if err := json.Unmarshal(raw, &programs); err != nil {
		return err
	}
	if len(programs) == 0 {
		return errors.New("empty NC package")
	}
	seen := make(map[string]bool)
	for _, p := range programs {
		if !programName.MatchString(p.Filename) || seen[p.Filename] || p.Text == "" || len(p.Errors) != 0 {
			return errors.New("invalid NC program")
		}
		seen[p.Filename] = true
	}
	file, err := os.CreateTemp(dir, "programs-*.zip.tmp")
	if err != nil {
		return err
	}
	defer os.Remove(file.Name())
	archive := zip.NewWriter(file)
	for _, p := range programs {
		member, err := archive.Create(p.Filename)
		if err == nil {
			_, err = member.Write([]byte(p.Text))
		}
		if err != nil {
			file.Close()
			return err
		}
	}
	if err = archive.Close(); err != nil {
		file.Close()
		return err
	}
	if err := file.Close(); err != nil {
		return err
	}
	return os.Rename(file.Name(), filepath.Join(dir, "programs.zip"))
}
