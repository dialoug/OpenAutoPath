# Build OpenAutoPath

OpenAutoPath runs as a browser application. The Go server serves the built web assets and starts a native C++ worker for CAM tasks.

## Prerequisites

- CMake 3.22 or newer, Git, and a compiler and standard library with C++20 support. The VS 2026 preset also needs a CMake version that supports its generator.
- OCCT 7.9.3. Install the x64 release SDK on Windows with `tools/Install-WindowsOcct.ps1`. Linux and macOS use the native OCCT CMake package.
- Go 1.23 or newer and Node.js 22 with npm for the Web application.

CMake retrieves pinned Clipper2, OpenCAMLib, Boost and nlohmann/json sources on the first configure. For offline builds, provide local sources through the cache variables `OAP_CLIPPER2_ROOT`, `OAP_OPENCAMLIB_ROOT`, `OAP_OPENCAMLIB_BOOST_ROOT`, and `OAP_JSON_ROOT`. Windows SDK paths can be set with `OAP_OCCT_ROOT` and `OAP_THIRDPARTY_ROOT`. Keep machine-specific paths in `CMakeUserPresets.json`.

## Native worker and tests

| Platform | Preset | Configuration |
| --- | --- | --- |
| Windows x64, VS 2026 | `windows-core-vs2026` | Release |
| Windows x64, VS 2022 | `windows-core` | Release |
| Linux | `linux-core` | RelWithDebInfo |
| macOS | `macos-core` | RelWithDebInfo |

Run the Windows commands from a Visual Studio Developer PowerShell if CMake is only available through Visual Studio.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Install-WindowsOcct.ps1
cmake --preset windows-core-vs2026
cmake --build --preset windows-core-vs2026
ctest --preset windows-core-vs2026
ctest --preset windows-core-vs2026-packaging
```

The full C++ build includes the worker and test executables. To build the worker alone, append `--target oap-worker`. The regular test preset runs the fast tests. For the full suite, run `ctest --test-dir build/windows-core-vs2026 -C Release --output-on-failure`.

On Linux or macOS, the repository provides a [micromamba environment](.github/ci/core-environment.yml) matching CI. Use the corresponding native preset and supply `-DCMAKE_PREFIX_PATH="$CONDA_PREFIX"` when using that environment.

## Run the Web application

Follow the [server build and startup instructions](server/README.md) to assemble the native worker, Go server, and browser assets. The server listens on `http://127.0.0.1:8080` by default. Keep its writable data directory outside the package so upgrades retain existing models and jobs.

`ServerWorker` installs the worker and its native runtime. `Development` installs libraries, headers and CMake exports for C++ consumers. On Linux or macOS, keep the platform's OCCT runtime available on the host. For third-party licenses, see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
