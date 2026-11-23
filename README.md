# OpenAutoPath

[简体中文](README.zh-CN.md) · [Build and run](server/README.md) · [Architecture](ARCHITECTURE.md)

OpenAutoPath is a browser-based CAM workbench. Upload a STEP model, inspect its recognized features, and create machining operations for holes, pockets, profiles, and engraving. Edit tool libraries and machining projects, preview toolpaths and remaining stock in 3D, and review reference NC before downloading it.

A Go HTTP server serves the interface and manages model, project, and task versions. A native C++ worker runs geometry, toolpath, and simulation calculations. The browser client uses React and Three.js.

- [Build prerequisites and native tests](BUILDING.md)
- [Try the CAM sample model](samples/README.md)
- [HTTP contract](contracts/http/openapi.yaml)
- [License](LICENSE) and [third-party notices](THIRD_PARTY_NOTICES.md)
