# Building OpenAutoPath

CMake 3.22 or newer and C++20 are required. Shared presets are the entry point
for the command line, VS Code CMake Tools and CI. Generated `.vcxproj` files
are build outputs: edit CMake source files instead.

## Platform scope

| Platform | Build | Preset |
| --- | --- | --- |
| Windows x64, MSVC / VS 2022 | Desktop and core | `windows-vs2022` |
| Windows x64, MSVC / VS 2026 | Desktop and core | `windows-vs2026` |
| Windows x64, MSVC | Core only | `windows-core` / `windows-core-vs2026` |
| Linux | Core only | `linux-core` |
| macOS | Core only | `macos-core` |

Windows desktop builds are locally validated. Linux/macOS have native build,
test and package-consumer CI jobs; check their results on those runners.
Desktop GUI and deployment are currently Windows-only. All builds use Qt 6.11.1
and OCCT 7.9.3 as their dependency baseline. The GUI uses Qt private headers:
build and deploy it with the same Qt installation. Core-only builds still need
Qt Core, OCCT, Clipper2, OpenCAMLib and Boost.

## Windows dependencies

The default local layout is:

```text
external/
  opencascade-7.9.3-vc14-64/
  3rdparty-vc14-64/
  Clipper2/
  opencamlib/
  boost_1_87_0/
```

`external/` is ignored by Git. Copy an existing bundle there, or download
the official OCCT SDK and runtime bundle with SHA256 verification:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Install-WindowsOcct.ps1
```

The installer preserves existing SDK directories and does not install Qt.
Qt defaults to `C:/Qt/6.11.1/msvc2022_64` on Windows.

Clipper2 and OpenCAMLib use pinned Git revisions when no local checkout exists:
`21ebba05db8894f0c7217ad35ea518080f324946` and
`1a450b7c3d3b669224a2a5d09aab7f319794574c` respectively. Local checkouts should
match these revisions. Boost 1.87.0 and, for GUI builds, Vulkan headers are
downloaded with SHA256 verification when absent. The first configuration then
needs network access; fetched sources normally live under the build's `_deps/`.

Dependency roots accept CMake cache arguments or environment variables:
`OAP_QT_ROOT`, `OAP_OCCT_ROOT`, `OAP_THIRDPARTY_ROOT`, `OAP_CLIPPER2_ROOT`,
`OAP_OPENCAMLIB_ROOT` and `OAP_OPENCAMLIB_BOOST_ROOT`. Cache arguments take
precedence. Changing an environment variable does not replace a cached path.

## Configure, build and test

For the local VS 2026 installation:

```powershell
cmake --preset windows-vs2026
cmake --build --preset windows-vs2026
ctest --preset windows-vs2026
ctest --preset windows-vs2026-packaging
./build/windows-vs2026/RelWithDebInfo/OpenAutoPath.exe
```

Use `windows-vs2022` for VS 2022. Desktop presets build RelWithDebInfo;
Windows core presets build Release. Test presets select `fast`;
presets with the `-packaging` suffix select the installed-package check.
The shared parallel limit is four. With sufficient memory, configure with
`-DOAP_BUILD_PARALLEL_JOBS=8` and build with `--parallel 8`.

The Windows OCCT SDK contains **release binaries only**. Supported configurations
are RelWithDebInfo, Release and MinSizeRel. Debug and custom configurations are
rejected at configure time. Use RelWithDebInfo for symbols. The `PACKAGE`
provider can support Debug when native dependencies provide compatible binaries;
Windows packages must export Debug import libraries.

To build only the application, append `--target OpenAutoPath`. Tests need their
executables built, so use the default build before CTest. GPU tests are separate
and require a suitable Vulkan device. Existing build directories remain usable:

```powershell
cmake -S . -B build
cmake --build build --config RelWithDebInfo --parallel 8
ctest --test-dir build -C RelWithDebInfo -L fast --output-on-failure
```

Do not change generators in an existing build directory. Presets use separate
directories under `build/<preset>/`. Outputs within each build are under
`<configuration>/`: launcher at the root, application/DLLs in `bin/`, archives
in `lib/`, symbols in `symbols/` and tests in `tests/`.

## Linux and macOS core

The native provider uses `find_package(OpenCASCADE 7.9.3 EXACT CONFIG)` and
Qt discovery through `CMAKE_PREFIX_PATH`. It does not use Windows libraries
from `external/`. OCCT 8 is not API-compatible with this baseline.

For the same dependency environment as CI, install micromamba, then:

```sh
micromamba create -f .github/ci/core-environment.yml
micromamba run -n oap-core cmake --preset linux-core
micromamba run -n oap-core cmake --build --preset linux-core
micromamba run -n oap-core ctest --preset linux-core
micromamba run -n oap-core ctest --preset linux-core-packaging
```

Use `macos-core` on macOS. If discovery needs an explicit prefix, activate the
environment and pass `-DCMAKE_PREFIX_PATH="$CONDA_PREFIX"` to configure, as CI
does. System packages also work with the same versions and compatible
compiler/architecture. The environment pins Qt and OCCT; transitive dependencies
and compiler versions are resolved by conda-forge, not a fully locked environment.

## Options and local presets

- `OAP_BUILD_GUI=OFF` omits app/presentation targets and their tests.
- `OAP_DEPLOY_RUNTIME=OFF` skips Windows desktop DLL copying and windeployqt.
  Launching that GUI needs SDK DLL search paths and Qt plugins. CTest configures
  its own paths and does not depend on GUI deployment.
- `OAP_OCCT_PROVIDER=WINDOWS_SDK` uses the bundle. `PACKAGE` uses upstream
  OpenCASCADE targets; set `OpenCASCADE_DIR` if necessary.
- `BUILD_TESTING=OFF` disables test targets and registrations.
- `OAP_INSTALL_DEVELOPMENT=OFF` omits library/header/package installation.

`CMakePresets.json` is tracked. Put personal SDK paths in ignored
`CMakeUserPresets.json`, for example:

```json
{
  "version": 3,
  "configurePresets": [{
    "name": "my-windows",
    "inherits": "windows-vs2026",
    "cacheVariables": { "OAP_QT_ROOT": "D:/Qt/6.11.1/msvc2022_64" }
  }],
  "buildPresets": [{
    "name": "my-windows", "configurePreset": "my-windows",
    "configuration": "RelWithDebInfo", "jobs": 4
  }],
  "testPresets": [{
    "name": "my-windows", "inherits": "windows-vs2026",
    "configurePreset": "my-windows"
  }]
}
```

VS Code CMake Tools uses presets via repository settings. Choose the preset
matching your installed Visual Studio version or host platform.

## Install and consume

After building, install using the matching directory and configuration:

```powershell
cmake --install build/windows-vs2026 --config RelWithDebInfo
```

The preset installs into `out/install/windows-vs2026`. Override with
`--prefix <directory>`. Desktop installs contain `OpenAutoPath.exe` plus
`bin/OpenAutoPathApp.exe` and deployed DLLs/plugins when deployment is on.
`--component Runtime` selects desktop files; `--component Development` selects
core libraries, headers and CMake exports. OpenCAMLib remains a replaceable
shared library. Licenses and notices are installed under `share/`.

An independent C++ project can use:

```cmake
find_package(OpenAutoPath CONFIG REQUIRED)
add_executable(example main.cpp)
target_link_libraries(example PRIVATE OAP::Analysis OAP::PocketToolpath)
```

Set `CMAKE_PREFIX_PATH` to install and dependency prefixes, or set
`OpenAutoPath_DIR` to `<prefix>/lib/cmake/OpenAutoPath`. Windows SDK consumers
also provide `OAP_OCCT_ROOT`, discoverable Qt and release-compatible
`CMAKE_CONFIGURATION_TYPES` (e.g. RelWithDebInfo). For native packages use
`OAP_OCCT_PROVIDER=PACKAGE`. OCCT and Qt remain external dependencies; exports
do not embed the producer's dependency paths. Consumers must use compatible
architecture, compiler ABI and runtime configuration.

Development installs use the core public-header manifest rather than copying
entire source directories. `cmake/OAPSdkHeaders.cmake` explicitly lists any
additional support headers required by public types. These support headers do
not expand the permitted cross-module dependencies in application source code.
The dependency closure is checked at configure time and in architecture CI.

The `packaging` test clears its own staging directory inside the build tree,
installs development files, and verifies that the installed first-party header
set exactly matches the SDK manifest. An independent consumer compiles each
installed header separately, then builds and runs a geometry/hash smoke test
using OCCT, Qt and OpenCAMLib. Build all core targets first. The exported API
has no package version or ABI stability promise yet. OpenCAMLib is an
implementation dependency; its raw headers are not part of the exported API.

## Build layout and CI

`CMakeLists.txt` declares targets and module relationships. Shared helpers stay
under `cmake/`; platform policy and Windows runtime handling are under
`cmake/platforms/`; OCCT discovery is under `cmake/dependencies/`.
`cmake/OAPInstall.cmake` owns installation and exports. CMake generates Visual
Studio or Ninja projects; a handwritten Makefile is unnecessary.

Windows CI uses desktop RelWithDebInfo and core Release presets. Linux/macOS CI
use their core presets with a conda-forge environment. All build selected
targets, run fast tests and verify the installed package. Fetched sources live
outside the checkout in CI, exercising external-source grouping. Architecture
CI separately checks source boundaries and documentation.
