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
| FreeImage | 3.18.0 源码，含内部编解码库 | 原发行链接见下文；源码归档通过 MacPorts 镜像取得，并固定 SHA-256；保留上游 VS 工程与构建文件 |
| FFmpeg | 3.3.4 官方源码包 | 当前 OCCT SDK 构建参数见下文；未启用 GPL 或 nonfree |
| FreeType | 2.13.3 官方源码包 | 包含 FTL 及内部组件声明；Qt 内置的另一版本随 qtbase 源码提供 |

## 构建与库替换

Qt、OCCT、OpenCAMLib、FFmpeg 都以可替换 DLL 使用。使用者可以为自身用途修改库，
并进行调试这些修改所需的逆向工程；项目不对这些 LGPL 权利附加限制。
替换 DLL 时须保持架构、ABI 和运行库兼容；Qt 私有头文件的使用使本项目依赖具体 Qt 构建。
OpenAutoPath 不使用阻止用户替换 DLL 的签名校验或安装锁。

源码仓库的 `BUILDING.md`、CMake 配置及 `tools/Install-WindowsOcct.ps1` 描述应用构建
和 SDK 获取方式。Qt 的原始构建信息位于 `share/licenses/OpenAutoPath/build-evidence`。
FFmpeg SDK 的 `ffprobe -L` 报告如下配置：

```text
--enable-shared --disable-static --prefix=/d/build_ffmpeg/ffmpeg-3.3.4-64
--toolchain=msvc --arch=amd64 --disable-asm --disable-yasm --disable-swresample
```

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

源码归档是固定版本的上游源文件，不是“已经复现出相同二进制”的证明。OCCT 官方
第三方预编译 SDK 未随包提供所有构建脚本和补丁说明；目前未验证每个 DLL 与上游源码
的逐项对应关系。特别是 FreeImage/FFmpeg 等供应商构建，仍应取得或复现实际构建所需
的补丁和配置，才能完成对完整对应源码的核实。本项目不会把版本相同或文件齐全等同于
这项核实已经完成。替换 SDK 或新增依赖时，需重新核对并更新许可清单，不能只改哈希。
