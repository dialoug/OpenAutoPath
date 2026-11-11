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

The desktop package targets Windows 10 1809 or later and Windows 11 (x64),
matching the [Qt 6.11 Windows baseline](https://doc.qt.io/qt-6.11/windows.html).
Deployment uses `windeployqt --no-system-d3d-compiler`: `D3Dcompiler_47.dll`
is loaded from Windows, not copied from the build machine's SDK. The isolated
package test verifies the system DLL and its `D3DCompile` entry point.
See [Microsoft's system-component explanation](https://devblogs.microsoft.com/cppblog/on-c-amp-remappable-shader/).

中文说明：桌面包面向 Windows 10 1809 及以上版本和 Windows 11（x64）。
`D3Dcompiler_47.dll` 使用系统自带版本，不随包复制开发机的 SDK 版本；
独立测试包验证会检查系统 DLL 及其编译入口能否加载。旧构建目录若残留该 DLL，
请先将它移到运行目录之外再打包；分发检查仍会拒绝未登记的文件。

Deployment also excludes optional SVG plugins (`qsvg`, `qsvgicon`) and the Qt
SDK's `opengl32sw.dll` software rasterizer. The application has no SVG assets;
3D rendering uses the system's graphics driver. This keeps CI packages aligned
with the locally validated runtime, regardless of extra Qt modules on the host.

中文说明：部署时也排除未使用的 SVG 插件及 Qt SDK 的 OpenGL 软件渲染库，
避免 CI 因安装了额外 Qt 模块而扩大分发范围。三维显示使用系统图形驱动。

DXC (`dxcompiler.dll`, `dxil.dll`) is deployed from the pinned Microsoft
`v1.9.2602` archive, with SHA-256 checks for both the archive and x64 DLLs.
CMake downloads it for Windows desktop deployment; offline builds can set
`OAP_DXC_ROOT` to an extracted `dxc_2026_02_20.zip`. The host Windows SDK's DXC
is not copied by `windeployqt`.

中文说明：DXC 固定使用已核对的微软 `v1.9.2602` 版本，归档和两个 x64 DLL
都校验 SHA-256，不再随构建机 Windows SDK 的版本变化。离线构建可通过
`-DOAP_DXC_ROOT=解压目录` 指定同一归档；版本不符时配置会直接失败。

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

For desktop deployment, build FreeImage and OCCT with video support disabled,
following [dependency source instructions](licenses/DEPENDENCY_SOURCES.md).
Pass their roots as `-DOAP_FREEIMAGE_ROOT=.../build/dependencies/freeimage`
and `-DOAP_OCCT_ROOT=.../build/dependencies/occt/install` to the desktop preset.
The vendor OCCT SDK remains usable for core-only builds and tests.

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
cmake --preset windows-vs2026 -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage" -DOAP_OCCT_ROOT="$PWD/build/dependencies/occt/install"
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

## Windows release delivery / Windows 自动打包与发布

`.github/workflows/release.yml` reuses all three CI workflows for the same
commit. Only after architecture checks, Windows desktop/core tests and SDK
checks, and Linux/macOS core tests and SDK checks pass can it publish a release.
The Windows desktop package uses the tested RelWithDebInfo binaries, without
PDBs or development libraries. It includes the launcher, application, deployed
DLLs, Qt plugins, runtime installer, existing third-party notices and build ID.
Linux/macOS currently validate the core only; this workflow does not publish
desktop packages for those platforms.

**中文说明：** `.github/workflows/release.yml` 会针对同一个提交复用三套 CI。
只有架构检查、Windows 桌面版与核心版测试、Linux/macOS 核心版测试，以及各平台的
SDK 安装验证全部通过，才会发布。当前发布产物仅为 Windows x64 桌面 ZIP，使用经过
测试的 RelWithDebInfo 二进制，不包含 PDB 调试符号或开发库；包内包含启动器、应用、
运行库、Qt 插件、运行库安装程序、现有第三方声明和构建标识。
Linux/macOS 目前只验证核心模块，不生成桌面发布包。

First push the workflow changes to GitHub. Run **Release** manually in Actions
to build a preview ZIP and `SHA256SUMS.txt` under the
`windows-desktop-package` artifact without creating a GitHub Release. Manual
runs never publish, including runs selected on an existing tag.

**先试打包：** 将这些工作流文件提交并推送到 GitHub 后，在仓库的
**Actions → Release → Run workflow** 中手动运行。完成后，在该次运行页面的
**Artifacts（构建产物）** 中下载 `windows-desktop-package`，其中包含 ZIP 和
`SHA256SUMS.txt`。手动运行不会创建或发布 GitHub Release，即使选择的是已有标签。
分支上的手动运行使用 `preview-提交短哈希` 命名；标签上的手动运行沿用标签名称。

To publish a version, tag the intended commit and push that tag:

**正式发布：** 先确认当前提交就是要发布的版本，再创建并推送版本标签。
下面的 `v0.1.0` 是示例，实际使用时替换为本次版本号：

```powershell
git tag -a v0.1.0 -m "OpenAutoPath v0.1.0"
git push origin v0.1.0
```

Tags must be `vMAJOR.MINOR.PATCH` with an optional prerelease suffix such as
`v0.1.0-rc.1`. Suffix versions become GitHub prereleases. The workflow uses
the built-in `GITHUB_TOKEN`; only the publishing job has `contents: write`.
It verifies the downloaded ZIP's checksum, creates a draft, uploads the assets,
then publishes. A failed upload can resume on rerun;
an already published version is not overwritten. Publish fixes under a new tag.
Repository or organization policy must allow the publishing job's token scope.

**版本与权限：** 标签格式为 `v主版本.次版本.修订版本`，例如 `v0.1.0`；
也可以使用 `v0.1.0-rc.1` 等后缀，带后缀的版本会标记为预发布版。
流程使用 GitHub 内置的 `GITHUB_TOKEN`，无需另填个人访问令牌；只有发布任务申请
`contents: write` 写权限，仓库或组织策略须允许该权限。

**失败与重试：** 发布任务先校验下载的 ZIP，再创建 Release 草稿、上传 ZIP 和校验文件，
最后公开该 Release（可见范围遵循仓库权限）。上传中断后可以重新运行流程，继续处理草稿；
已经发布的版本不会被覆盖，后续修复应使用新标签。SHA-256 用于检查文件完整性，
构建标识保存在包内的 `build-info.json` 中。

The ZIP is unsigned. Extract it completely, install `bin/vc_redist.x64.exe` if
needed, and launch the root `OpenAutoPath.exe`. File-presence checks validate
the runtime layout; they are not a clean-machine interactive GUI test.

**下载后运行：** 完整解压 ZIP，保留 `bin` 和 `share` 的相对位置，运行根目录的
`OpenAutoPath.exe`。如果系统尚未安装 Visual C++ 运行库，先运行
`bin/vc_redist.x64.exe`。当前产物没有代码签名；打包阶段检查必要文件是否齐全，
不等同于已经在未安装开发环境的干净 Windows 系统上完成交互式 GUI 测试。

To package an existing tested local desktop build into a fresh output folder:

**本地打包：** 先构建并测试对应桌面版本，再指定构建目录和新的输出目录运行：

```powershell
./tools/Package-WindowsRelease.ps1 -BuildDir build/windows-vs2026 `
  -Version v0.1.0-rc.1 -OutputDir artifacts/release-local
```

The packager runs `cmake --install --component Runtime` into a unique stage,
checks required runtime files and writes the ZIP, SHA-256 and build metadata.
The recorded commit identifies the source checkout; build and test that checkout
before packaging. Release CI always builds a fresh checkout of the selected ref.

**参数说明：** `-BuildDir` 指向已有的 CMake 构建目录，`-Version` 是产物版本名称，
`-OutputDir` 是 ZIP 和校验文件的输出目录；`-Configuration` 默认为
`RelWithDebInfo`，也可指定已构建的 `Release` 或 `MinSizeRel`。
脚本不会重新编译，而是通过 `cmake --install --component Runtime` 将运行文件安装到
新的临时目录，检查必要文件后压缩并计算 SHA-256。输出目录若已有同名 ZIP 或
`SHA256SUMS.txt`，脚本会拒绝覆盖，应改用新的输出目录。
记录的提交号来自当前源码目录，因此本地打包前应保证二进制与该提交对应；
云端发布流程会从所选提交重新检出并构建。

## License and redistribution / 许可与再分发

Project-owned code and documentation are MIT-licensed; see [LICENSE](LICENSE).
Third-party components and adapted code retain their original terms; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Both Runtime and Development installations include these documents and the
AnalysisSitus BSD notice under `share/licenses/OpenAutoPath`, alongside the
existing OpenCAMLib, Boost and Clipper2 notices. Reconfigure an existing build
directory before installing to pick up the updated installation rules.

项目自有代码和文档采用 MIT。第三方依赖和移植代码保留原许可，不能将整个安装包
都视为 MIT。Runtime 与 Development 安装均会携带上述许可文件；旧构建目录应先
重新运行 CMake 配置，再安装或打包。打包脚本会检查这些文件是否存在。
Windows Runtime 还安装 Qt、OCCT 及附带运行库的声明。打包脚本逐个核对实际 DLL，
输出组件、版本、SHA-256 和许可路径清单，并保留 Qt SDK 的 SPDX 和构建配置。
未知 DLL、已审核 SDK 二进制变化或许可文件缺失会阻止打包；更换 SDK 时需重新审核
`licenses/windows-runtime.json`，不能仅更新哈希以跳过检查。

打包还会下载并校验五份依赖源码归档，存入 ZIP 的 `share/sources/OpenAutoPath`。
默认下载缓存位于构建目录的 `release-source-cache`；可用 `-SourceCache 路径` 复用缓存，
加 `-OfflineSources` 则只允许使用已缓存且哈希匹配的归档。普通 CMake 配置、构建和
安装不会触发这些下载；发布 CI 按清单缓存归档。源码归档会增加 ZIP 体积。

桌面 CI 从固定源码构建 FreeImage 和 OCCT，并保留构建记录；Qt 及其余预编译
依赖的核对范围见 [依赖源码说明](licenses/DEPENDENCY_SOURCES.md)。文件校验不是外部签名认证。
已发布的 ZIP 不会因源码新增许可文件而自动更新。

The Windows packager validates deployed libraries against `licenses/windows-runtime.json`,
includes upstream notices and Qt build evidence, and bundles five hash-verified dependency
source archives. `-SourceCache` selects a reusable cache; `-OfflineSources` disables downloads.
Ordinary CMake configure/build/install remains offline with respect to these source archives.
Vendor binary/source correspondence limitations are documented in `licenses/DEPENDENCY_SOURCES.md`.

**Windows 依赖源码构建：** 桌面 CI 构建 FreeImage 3.18.0 和 OCCT 7.9.3。
FreeImage 应用仓库中的 OpenEXR 兼容补丁并验证 PNG 读写；OCCT 不修改上游源码，关闭 FFmpeg，使用 NATIVE 分配器。
源码 SDK 有独立缓存，首次构建较慢。Windows 构建作业上限为 120 分钟。

桌面配置必须用 `OAP_FREEIMAGE_ROOT`、`OAP_OCCT_ROOT` 指向这两套源码 SDK，具体命令见
[依赖源码说明](licenses/DEPENDENCY_SOURCES.md)。供应商 SDK 保留给核心开发/测试，不能用于桌面部署。
旧构建目录需移走 FFmpeg 四个 DLL 和 jemalloc，建议采用新的构建目录。

`v*` 版本要求两份源码构建记录匹配。所有桌面包都会拒绝 FFmpeg DLL、启用视频的 OCCT 构建记录、
缺失/混用依赖和残留 jemalloc。配套脚本、补丁和五份源码归档随包提供。

The desktop workflow builds and caches pinned FreeImage and OCCT sources. OCCT uses
`USE_FFMPEG=OFF` and the native allocator. Desktop deployment requires this SDK;
the vendor SDK remains available for core-only tests. Versioned packages require
matching build records for both libraries. FFmpeg is not shipped.

### 独立测试包验证

手动运行 Release 工作流生成测试包，不创建正式版本标签。构建成功后，另一个 Windows runner
下载 ZIP，校验完整性、五份源码归档、两份构建记录和全部 DLL/EXE 的直接及延迟导入表，确认没有 FFmpeg。
随后清除 Qt/QML 环境变量，将 PATH 限制为 Windows 系统目录，从空工作目录启动包内启动器，
检查主窗口响应以及 Qt/OCCT 等模块确实来自包内。报告上传为 `windows-package-validation`。

这个独立 runner 没有执行 Qt/OCCT SDK 安装步骤，但 GitHub runner 本身带有开发工具和系统运行库，
因此不能称为纯净 Windows 安装，也不能代替人工检查 STEP 显示和加工界面。
真正的纯净系统交互验证仍需 Windows Sandbox 或专门的虚拟机。
