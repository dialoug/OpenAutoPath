# 依赖源码与重新构建

Windows 发布 ZIP 的 `share/sources/OpenAutoPath` 随包提供下列上游源码归档。
文件名、下载地址及 SHA-256 记录在 `share/licenses/OpenAutoPath/windows-runtime.json`
和 `runtime-inventory.json` 中；打包时校验哈希，下载失败或哈希不符会停止打包。
这些文件也可按源码仓库中的 [windows-runtime.json](windows-runtime.json) 独立下载。

| 组件 | 源码 | 构建信息 |
| --- | --- | --- |
| Qt | qtbase 6.11.1 官方源码包 | 随包保留 SDK 的 SPDX 清单及 `config_qtbase.opt`、`config_qtbase.summary`；清单记录源码提交 `59c81a3c2247b821b9b84b4eb8d939b77e07e276` 和编译器信息 |
| OCCT | `V7_9_3` 标签源码 | Windows 二进制来自 Open-Cascade-SAS 官方发布的 `opencascade-7.9.3-vc14-64.zip`；获取与校验见 `tools/Install-WindowsOcct.ps1` |
| OpenCAMLib | `1a450b7c3d3b669224a2a5d09aab7f319794574c` | 项目 `cmake/OAPOpenCamLib.cmake` 提供构建封装，生成共享库，关闭内部 OpenMP；上游 C++ 源码未修改 |
| FreeImage | 3.18.0 源码，含内部编解码库 | 桌面 CI 使用本项目构建脚本从固定归档构建，并应用一个 OpenEXR 标准库兼容补丁；随包保留构建记录及可重用脚本 |
| FFmpeg | 3.3.4 官方源码包 | 当前 OCCT SDK 构建参数见下文；未启用 GPL 或 nonfree |
| FreeType | 2.13.3 官方源码包 | 包含 FTL 及内部组件声明；Qt 内置的另一版本随 qtbase 源码提供 |

## 构建与库替换

Qt、OCCT、OpenCAMLib、FFmpeg 都以可替换 DLL 使用。使用者可以为自身用途修改库，
并进行调试这些修改所需的逆向工程；项目不对这些 LGPL 权利附加限制。
替换 DLL 时须保持架构、ABI 和运行库兼容；Qt 私有头文件的使用使本项目依赖具体 Qt 构建。
OpenAutoPath 不使用阻止用户替换 DLL 的签名校验或安装锁。

源码仓库的 `BUILDING.md`、CMake 配置及 `tools/Install-WindowsOcct.ps1` 描述应用构建
和 SDK 获取方式。Qt 的原始构建信息位于 `share/licenses/OpenAutoPath/build-evidence`。
已直接调用所部署 FFmpeg 四个 DLL 的配置导出函数（avcodec、avformat、avutil、swscale），
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
打包允许已核实的原供应商 DLL，或带匹配构建记录的源码构建 DLL；不能通过只改 DLL
哈希来接纳新的源码构建。自建记录提供本项目构建链的可追溯信息，不是外部签名证明。

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

FreeImage 的桌面 CI 路径已改为受记录的源码构建；OpenCAMLib 本来就由项目构建。
FFmpeg 的实际编译参数已从 DLL 核实，但仍未复现其工具链和供应商补丁。OCCT 及其他
预编译依赖也没有全部重新构建；jemalloc SDK 头文件的版本宏为
`0.0.0-0-g000000missing_version_try_git_fetch_tags`，不能据此确定实际上游提交。
这些剩余项仍需供应商构建材料或受控源码构建，不能将版本相同、哈希一致或文件齐全
视为完整对应源码已经核实。更换 SDK 或新增依赖时需重新核对。
