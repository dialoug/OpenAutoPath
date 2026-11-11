# OpenAutoPath

**English** | [简体中文](README.zh-CN.md)

OpenAutoPath is a CAD/CAM desktop project built with **C++20, Qt, and Open CASCADE (OCCT)**.
It provides STEP model viewing, machining feature recognition, toolpath planning,
material removal simulation, and reference NC program export.

The project currently provides a **Windows x64 desktop application**. Core modules
are built and tested on Linux and macOS, but desktop release packages are not yet available for those platforms.

[Download for Windows](https://github.com/dialoug/OpenAutoPath/releases/latest) ·
[Build instructions](BUILDING.md) · [Architecture](ARCHITECTURE.md) ·
[CI and release runs](https://github.com/dialoug/OpenAutoPath/actions) ·
[Report an issue](https://github.com/dialoug/OpenAutoPath/issues)

## Features

| Feature | Current capabilities |
| --- | --- |
| Model viewing and analysis | Import STEP models, inspect geometry and topology, and select and analyze model elements |
| Machining feature recognition | Identify machining regions associated with holes, pockets, and exterior profiles for subsequent planning |
| Hole machining | Generate fixed-axis drilling and hole milling toolpaths |
| Pocket machining | Generate offset, zigzag, spiral, and FreePath clearing paths with entry moves and links |
| Exterior profile machining | Generate fixed-axis exterior clearing and profile finishing paths |
| Surface engraving | Use OpenCAMLib for fixed-axis raster machining with ball, flat, and bull-nose cutters; run finishing alone or stock-based layer roughing followed by finishing |
| Manufacturing projects | Save operations, setups, tools, and parameters; rebuild projects and organize toolpaths by operation |
| Stock simulation | Compute in-process workpieces (IPW) in operation order, display material removal, and replay toolpaths |
| NC export | Export reference NC through the project postprocessor, representing path primitives as linear moves, applicable arcs, and drilling cycles |

Manufacturing projects use `.oapjob` files; tool libraries use `.oaptl` files.

## Download and launch

1. Open [Releases](https://github.com/dialoug/OpenAutoPath/releases) and choose a version.
2. Download the application package named `OpenAutoPath-<version>-windows-x64.zip` from the release assets.
   GitHub's automatically generated `Source code` archives contain source files, not a runnable application.
3. Extract the entire archive, preserving directories such as `bin` and `share` and their relative locations.
4. Run `OpenAutoPath.exe` at the root of the extracted directory.
5. If the Visual C++ runtime is missing, first run the bundled `bin/vc_redist.x64.exe`.

Releases include `SHA256SUMS.txt` for download integrity checks. The package's `build-info.json`
records the version, build configuration, and source commit; `README.txt` provides launch instructions in English and Chinese.
The ZIP package is currently unsigned.

## First steps

Start with a simple STEP model of known dimensions and work through the following steps:

1. **Open a model:** Import a `.step` or `.stp` file and check its display and dimensions.
2. **Define machining conditions:** Set the stock, workpiece coordinates, and setup orientation; choose tools and machining parameters.
3. **Choose a machining method:** Recognize the relevant features for hole, pocket, or exterior profile machining. Surface engraving can plan directly against the complete target model.
4. **Generate toolpaths:** Add the corresponding operations in the CAM workspace and inspect the results, entry and exit moves, and path links.
5. **Inspect remaining stock:** Use manufacturing project IPW to inspect material removal in operation order and replay the toolpaths.
6. **Save and export:** Save the manufacturing project. To export reference NC, first check the setup, tools, and program review results.

See the [surface engraving guide](src/toolpath/finishing/README.md) for its parameters and workflow entry points.
The repository includes [test models and usage instructions](samples/README.md). Download the STEP models
to try feature recognition, hole/pocket/profile machining, and surface engraving workflows.

## Current limitations

- **Platforms:** The desktop application supports Windows. Linux/macOS validation currently covers core modules and the installed SDK only.
- **Surface machining:** The current integration uses fixed-axis raster machining, not simultaneous five-axis machining. Waterline toolpaths are not yet integrated.
- **Remaining stock:** Layer roughing for surface engraving is planned from the stock envelope; scan paths are not yet clipped to the exact remaining material in preceding IPW states.
- **Simulation and NC:** Results depend on mesh quality, sampling, and simulation resolution. Full collision certification for shanks, tool holders, fixtures, and other components is not provided.
  NC output is a reference postprocessing result and must be validated for the target machine and controller before machining.
- **Developer interfaces:** Core libraries can be installed and consumed through CMake, but public API and ABI stability across versions is not yet guaranteed.

## Build from source

You need Git, CMake 3.22 or later, and a compiler supporting C++20.
The current dependency baseline is Qt 6.11.1 and OCCT 7.9.3. Core builds also require dependencies such as Qt Core.

### Windows desktop

Desktop builds require FreeImage and OCCT with FFmpeg disabled, built according to the
[dependency source instructions](licenses/DEPENDENCY_SOURCES.md). The original prebuilt OCCT SDK can be used for core tests.

Install Visual Studio 2022 with the Desktop development with C++ workload and Qt 6.11.1's
`msvc2022_64` components (including Qt Shader Tools), then run the following in PowerShell:

```powershell
git clone https://github.com/dialoug/OpenAutoPath.git
cd OpenAutoPath

# Download and verify the OCCT SDK; this script does not install Qt.
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Install-WindowsOcct.ps1

# First build FreeImage and OCCT without video support using the linked dependency source instructions.
# Replace the Qt path below if it is installed elsewhere.
cmake --preset windows-vs2022 -DOAP_QT_ROOT=C:/Qt/6.11.1/msvc2022_64 -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage" -DOAP_OCCT_ROOT="$PWD/build/dependencies/occt/install"
cmake --build --preset windows-vs2022
ctest --preset windows-vs2022
ctest --preset windows-vs2022-packaging

./build/windows-vs2022/RelWithDebInfo/OpenAutoPath.exe
```

For Visual Studio 2026, replace the preset name with `windows-vs2026` and update the corresponding
directory in the launch path. Initial configuration may require network access to fetch other pinned dependencies.
The Windows OCCT SDK used by the project provides release configurations only. Desktop presets use
`RelWithDebInfo`; switching directly to Debug is not supported.

### Linux / macOS core modules

Use the `linux-core` and `macos-core` presets, respectively. See [BUILDING.md](BUILDING.md)
for dependency installation, the micromamba environment used by CI, Qt/OCCT path settings, and installation instructions.

## Project layout and validation

| Directory or file | Contents |
| --- | --- |
| `src/app/` | Qt desktop UI and workflow coordination |
| `src/analysis/`, `src/recognition/` | Geometry analysis and feature recognition |
| `src/toolpath/` | Toolpath planning, validation, postprocessing, and material removal kernels |
| `src/manufacturing/` | Manufacturing projects, operations, and IPW lifecycle |
| `src/tooling/` | Tool libraries and machining parameter adapters |
| `tests/` | Automated tests for individual modules |
| `cmake/`, `CMakePresets.json` | Build targets, dependencies, tests, and installation rules |
| `.github/workflows/` | CI and release workflows |

Branch pushes and pull requests trigger architecture checks and Windows/Linux/macOS builds and tests.
Version tags trigger the Release workflow, which publishes a Windows package after all checks pass.
Manual Release runs only produce downloadable build artifacts; they do not create a formal release.
See the [Windows release instructions](BUILDING.md#windows-release-delivery--windows-自动打包与发布) for details.

Further reading: [Architecture](ARCHITECTURE.md), [toolpath modules](src/toolpath/README.md),
and [manufacturing projects](src/manufacturing/README.md).

## Issues and contributions

Please report problems through [GitHub Issues](https://github.com/dialoug/OpenAutoPath/issues).
Include the software version, operating system, reproduction steps, expected and actual results, and relevant logs or screenshots.
For geometry or toolpath issues, include a minimal model and parameters that you can share publicly;
do not upload models or business data you do not have permission to share.

Before submitting code changes, read the build and architecture documentation, run the relevant module tests,
and describe the reason for the change and validation results in your pull request.
A standalone contribution guide and issue templates are still to be added.

### Maintaining translated READMEs

`README.md` is the English landing page and `README.zh-CN.md` is the Simplified Chinese version; both live at the repository root.
When changing features, limitations, dependency versions, or build commands, update both versions in the same pull request.
Keep their section structure and technical information aligned; do not translate filenames, commands, parameters, or link targets.
Keep detailed build and architecture instructions in `BUILDING.md` and `ARCHITECTURE.md` rather than duplicating them in the READMEs.

## License

The project's own code and documentation are licensed under the [MIT License](LICENSE), with copyright attributed to `2026 dialoug`.
Third-party components and ported code retain their original licenses. The dihedral analysis code ported from AnalysisSitus
uses BSD-3-Clause. See [third-party notices](THIRD_PARTY_NOTICES.md) for dependency licenses,
the scope of package notices, and redistribution requirements.
