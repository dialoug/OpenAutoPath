# 依赖源码与重新构建

Windows 发布 ZIP 的 `share/sources/OpenAutoPath` 随包提供下列上游源码归档。
文件名、下载地址及 SHA-256 记录在 `share/licenses/OpenAutoPath/windows-runtime.json`
和 `runtime-inventory.json` 中；打包时校验哈希，下载失败或哈希不符会停止打包。
这些文件也可按源码仓库中的 [windows-runtime.json](windows-runtime.json) 独立下载。

| 组件 | 源码 | 构建信息 |
| --- | --- | --- |
| Qt | qtbase 6.11.1 官方源码包 | 随包保留 SDK 的 SPDX 清单及 `config_qtbase.opt`、`config_qtbase.summary`；清单记录源码提交 `59c81a3c2247b821b9b84b4eb8d939b77e07e276` 和编译器信息 |
| OCCT | `V7_9_3` 标签源码 | 桌面 CI 使用 `tools/Build-WindowsOCCT.ps1` 构建应用所需工具包，保留图像、视频、VR 和 TBB 支持；使用 NATIVE 内存分配器 |
| OpenCAMLib | `1a450b7c3d3b669224a2a5d09aab7f319794574c` | 项目 `cmake/OAPOpenCamLib.cmake` 提供构建封装，生成共享库，关闭内部 OpenMP；上游 C++ 源码未修改 |
| FreeImage | 3.18.0 源码，含内部编解码库 | 桌面 CI 使用本项目构建脚本从固定归档构建，并应用一个 OpenEXR 标准库兼容补丁；随包保留构建记录及可重用脚本 |
| FFmpeg | 3.3.4 官方源码包 | 桌面 CI 使用 `tools/Build-WindowsFFmpeg.ps1` 构建四个运行库和测试工具 ffprobe；未启用 GPL 或 nonfree |
| FreeType | 2.13.3 官方源码包 | 包含 FTL 及内部组件声明；Qt 内置的另一版本随 qtbase 源码提供 |

## 构建与库替换

Qt、OCCT、OpenCAMLib、FFmpeg 都以可替换 DLL 使用。使用者可以为自身用途修改库，
并进行调试这些修改所需的逆向工程；项目不对这些 LGPL 权利附加限制。
替换 DLL 时须保持架构、ABI 和运行库兼容；Qt 私有头文件的使用使本项目依赖具体 Qt 构建。
OpenAutoPath 不使用阻止用户替换 DLL 的签名校验或安装锁。

源码仓库的 `BUILDING.md`、CMake 配置及 `tools/Install-WindowsOcct.ps1` 描述应用构建
和 SDK 获取方式。Qt 的原始构建信息位于 `share/licenses/OpenAutoPath/build-evidence`。
已直接调用原供应商 FFmpeg 四个 DLL 的配置导出函数（avcodec、avformat、avutil、swscale），
四者均报告下列配置；av_version_info 报告 3.3.4，avcodec_license 报告 LGPL 2.1 或更高版本。
结果保存在 `windows-runtime.json` 的 `ffmpeg.runtimeBuildEvidence` 中：

```text
--enable-shared --disable-static --prefix=/d/build_ffmpeg/ffmpeg-3.3.4-64
--toolchain=msvc --arch=amd64 --disable-asm --disable-yasm --disable-swresample
```

## FreeImage 源码构建

桌面 CI 不再采用供应商预编译的 FreeImage.dll，而是执行
`tools/Build-WindowsFreeImage.ps1`。该脚本校验源码归档，在独立目录解压，使用上游
`FreeImage.2017.vcxproj` 构建 Release/x64，并运行 PNG 逐像素读写回归验证。
它不修改 `external/3rdparty-vc14-64` 中原有的 SDK。

唯一源码补丁删除 OpenEXR 比较器对已移除的 `std::binary_function` 的继承，保留原比较逻辑。
补丁位于 `tools/dependencies/freeimage-openexr-modern-cxx.patch`。实际编译器、工具集、
Windows SDK、命令参数、源归档/脚本/补丁/修改前后文件/DLL 的 SHA-256 均写入
`build-record.json`；安装后路径为 `share/licenses/OpenAutoPath/build-evidence/FreeImage-build-record.json`。

本地复现（在仓库根目录运行，输出目录必须尚不存在）：

```powershell
./tools/Build-WindowsFreeImage.ps1 `
  -SourceArchive artifacts/license-inputs/FreeImage3180-mirror.zip `
  -OutputDir build/dependencies/freeimage -Parallel 4
cmake -S . -B build -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage"
cmake --build build --config RelWithDebInfo
```

`SourceArchive` 可以使用随包源码目录中的同名归档。脚本自动选择 VS C++ 工具和 Windows SDK，
也可显式传入 `-VisualStudioRoot`、`-Toolset v143`（VS 2022）或 `v145`（VS 2026）、
`-WindowsSdkVersion`。包内 `share/sources/OpenAutoPath/OpenAutoPath-build-support` 保留
构建脚本、回归验证脚本、补丁及清单；进入该目录后也可执行相同构建命令。
本地 `preview-*` 打包允许已核实的原供应商 DLL，或带匹配构建记录的源码构建 DLL。
`v*` 版本打包要求 FreeImage、FFmpeg、OCCT 三者都有匹配的源码构建记录，不能通过只改 DLL
哈希来接纳新的源码构建。自建记录提供本项目构建链的可追溯信息，不是外部签名证明。

## FFmpeg 与 OCCT 源码构建

需要 Visual Studio C++ 工具、CMake、Git for Windows，以及清单固定的源码归档。
FFmpeg 使用 Git Bash 和 GNU Make。构建工具包为
`https://repo.msys2.org/msys/x86_64/make-4.4.1-2-x86_64.pkg.tar.zst`，SHA-256 为
`2408af61717dae87b00c855b132769a125c708907fc94a46bb16dae076113e5c`；脚本在使用前校验。
GNU Make 只参与构建，不进入应用运行时包。

先按上节构建 FreeImage，再在仓库根目录执行（输出目录应尚不存在）：

```powershell
./tools/Build-WindowsFFmpeg.ps1 `
  -SourceArchive artifacts/license-inputs/ffmpeg-3.3.4.tar.xz `
  -MakePackage artifacts/license-inputs/make-4.4.1-2-x86_64.pkg.tar.zst `
  -OutputDir build/dependencies/ffmpeg -Parallel 4
./tools/Build-WindowsOCCT.ps1 `
  -SourceArchive artifacts/license-inputs/OCCT-V7_9_3.tar.gz `
  -ThirdPartyRoot external/3rdparty-vc14-64 `
  -FreeImageRoot build/dependencies/freeimage `
  -FFmpegRoot build/dependencies/ffmpeg `
  -OutputDir build/dependencies/occt -Parallel 4
cmake -S . -B build `
  -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage" `
  -DOAP_FFMPEG_ROOT="$PWD/build/dependencies/ffmpeg" `
  -DOAP_OCCT_ROOT="$PWD/build/dependencies/occt/install"
cmake --build build --config RelWithDebInfo
```

OCCT 默认生成器为 VS 2022；VS 2026 传入 `-Generator "Visual Studio 18 2026"`。
`-ReuseExistingBuild` 可用于同一输出目录的增量构建，但脚本先将已有源码逐文件与原始归档比较，
不接受修改或新增的源文件。脚本限制编译并发，避免 MSBuild 与 `/MP` 叠加产生过多编译进程。

FFmpeg 保留 avcodec、avformat、avutil、swscale，关闭未部署的 avfilter、avdevice、
swresample 及播放器等程序，不使用 GPL/nonfree 配置或外部编解码库，也不修改上游源码。
ffprobe 用于 PPM 解码、数据包校验，另直接调用 swscale 检查 RGB/BGR 逐像素转换。
配置参数、编译器、SDK、输出 DLL 摘要写入记录；`config.h`、`config.mak` 随包保留。

OCCT 从未修改的上游源码构建，链接源码构建的 FreeImage 和 FFmpeg；FreeType、oneTBB、
OpenVR 继续使用已核实的官方 SDK 二进制。选择上游的 NATIVE 分配器，因此该路径不部署
版本来源不完整的 jemalloc。原供应商 SDK 本地配置仍保留 jemalloc，不能混用两种运行目录。

两者的 `build-record.json` 安装到 `build-evidence/ffmpeg-build-record.json` 和
`build-evidence/occt-build-record.json`。打包核对源码、脚本、DLL 摘要，另外核对 OCCT
记录的依赖 DLL；NATIVE 配置若残留 jemalloc 会停止打包。构建脚本与测试脚本随源码归档提供。
桌面 CI 缓存三个源码 SDK，源码清单、构建脚本或测试变更会改变缓存键。

## 其他原始来源和鸣谢

- OCCT 二进制及第三方 SDK：https://github.com/Open-Cascade-SAS/OCCT/releases/tag/V7_9_3
- FreeImage 原发行：https://sourceforge.net/projects/freeimage/files/Source%20Distribution/3.18.0/FreeImage3180.zip
- oneTBB：https://github.com/oneapi-src/oneTBB/tree/v2021.13.0
- OpenVR：https://github.com/ValveSoftware/openvr/tree/v1.14.15
- DirectX Shader Compiler：https://github.com/microsoft/DirectXShaderCompiler/tree/21d28f727
- jemalloc：https://github.com/jemalloc/jemalloc；随 OCCT SDK 的具体构建身份以二进制哈希为准，SDK 未提供可确认的版本号。

This software is based in part on the work of the Independent JPEG Group.
Portions of this software are copyright © 2024 The FreeType Project
(www.freetype.org). All rights reserved.
Qt 自身携带的 FreeType、JPEG 等组件另见 `runtime/Qt-6.11.1-NOTICES.txt`。
FreeImage 采用 FreeImage Public License；其内嵌 LibRaw 采用可选的 CDDL 许可，
各编解码库的原始声明位于 `runtime/FreeImage`，完整源码在 FreeImage 归档中。

## 核对边界

已按官方发布资产摘要校验两个 SDK 压缩包，并逐项确认现有 48 个 OCCT 及附带运行库
DLL 与其中的文件相同。归档地址、摘要、资产 ID 和对应条目记录在 `windows-runtime.json`。
这证明二进制分发来源，不能证明完整源码对应关系。

桌面 CI 的 FreeImage、FFmpeg、OCCT 路径使用本项目记录的源码构建；OpenCAMLib 本来就由项目构建。
这条路径不再依赖原供应商这三个 DLL 集合的未知构建过程，也不分发 jemalloc。
仍可选择的供应商 SDK 路径不能据此宣称已复现：尤其 jemalloc 的版本宏为
`0.0.0-0-g000000missing_version_try_git_fetch_tags`，不能确定其上游提交。

Qt 使用官方 SDK、对应源码归档、SPDX 清单与配置文件；FreeType、oneTBB、OpenVR 等
保留各自声明和已核实的分发来源，未全部在本项目重新构建。构建记录不是签名认证，
本流程也不声称不同工具链能产生逐字节相同的 DLL。更换 SDK、配置或新增依赖时需重新核对。
