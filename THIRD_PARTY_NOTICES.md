# 第三方组件与许可说明

OpenAutoPath 自有代码和文档采用 MIT，版权署名为 `Copyright (c) 2026 dialoug`，
完整条款见 [LICENSE](LICENSE)。第三方源码、移植代码、库、工具和运行时文件
保留各自的版权及许可证，不因根目录的 MIT 许可证而改为 MIT。

## 源码中的 AnalysisSitus 移植部分

`src/recognition/DihedralAngle.h` 和 `src/recognition/DihedralAngle.cpp` 中的
二面角判断、角度类型及辅助几何逻辑移植自 AnalysisSitus，保留 BSD-3-Clause 条款。
本项目移除了 ActAPI progress/plotter 等调试依赖，并适配了本地接口及缓存。

对应上游文件：

- [asiAlgo_CheckDihedralAngle.h](https://gitlab.com/ssv/AnalysisSitus/-/blob/master/src/asiAlgo/features/asiAlgo_CheckDihedralAngle.h)
- [asiAlgo_CheckDihedralAngle.cpp](https://gitlab.com/ssv/AnalysisSitus/-/blob/master/src/asiAlgo/features/asiAlgo_CheckDihedralAngle.cpp)
- [asiAlgo_FeatureAngleType.h](https://gitlab.com/ssv/AnalysisSitus/-/blob/master/src/asiAlgo/features/asiAlgo_FeatureAngleType.h)
- [asiAlgo_Utils.h](https://gitlab.com/ssv/AnalysisSitus/-/blob/master/src/asiAlgo/auxiliary/asiAlgo_Utils.h)

已按上游公开文件头核对署名和许可（2026-11-11）；这些链接用于核对许可，
不表示本项目移植时使用的精确上游提交。完整声明见
[licenses/AnalysisSitus-BSD-3-Clause.txt](licenses/AnalysisSitus-BSD-3-Clause.txt)。
安装时该文件与本文同放在 `share/licenses/OpenAutoPath` 中。

## 主要构建依赖

| 组件 | 当前配置或版本 | 使用方式与许可 |
| --- | --- | --- |
| [OpenCAMLib](https://github.com/aewallin/opencamlib) | `1a450b7c3d3b669224a2a5d09aab7f319794574c` | 独立共享库；LGPL-2.1-or-later；上游源码不改动，本项目提供构建封装并关闭其内部 OpenMP |
| [Clipper2](https://github.com/AngusJohnson/Clipper2) | `21ebba05db8894f0c7217ad35ea518080f324946` | 编译进项目；Boost Software License 1.0 |
| [Boost](https://www.boost.org/users/license.html) | 本地 SDK 为 1.87.0 | OpenCAMLib 使用其头文件；Boost Software License 1.0 |
| [Open CASCADE Technology](https://dev.opencascade.org/resources/licensing) | Windows SDK 为 7.9.3 | 动态链接；LGPL-2.1 with OCCT exception；保留 SDK 中的 LICENSE_LGPL_21.txt 和 OCCT_LGPL_EXCEPTION.txt |
| [Qt](https://www.qt.io/development/open-source-lgpl-obligations) | Windows CI 为 6.11.1 | Qt Core、Gui、Widgets、OpenGL、OpenGLWidgets 使用开源 LGPLv3 许可路径；Qt 所带第三方组件另有条款 |
| [Qt Shader Tools](https://doc.qt.io/qt-6.11/qtshadertools-index.html) | 同 Qt 版本 | qsb 是构建阶段使用的 GPLv3 工具；调用该工具不等于把应用自有源码改为 GPL |
| [Vulkan-Headers](https://github.com/KhronosGroup/Vulkan-Headers) | `v1.3.290` | 构建所需头文件；按对应文件的 Apache-2.0 或 MIT 等上游声明使用，不随此项引入 Vulkan loader |

实际依赖版本也可能由本地 CMake 配置覆盖，以构建缓存和随依赖提供的原始声明为准。
Runtime 与 Development 安装组件均携带本项目 MIT、本文、AnalysisSitus BSD 声明，
以及原有的 OpenCAMLib、Boost 和 Clipper2 许可文件。Windows Runtime 还安装
`licenses/runtime` 下的 Qt、OCCT 及下述运行时原始声明。

## Windows SDK 附带的运行时

部署列表还涉及 FreeType 2.13.3、FreeImage 3.18.0、oneTBB 2021.13.0、
FFmpeg 3.3.4、OpenVR 1.14.15、DirectX Shader Compiler 和 Microsoft Visual C++
运行库。这些文件不是 OpenAutoPath 自有代码。Qt Network 及平台、图像、TLS 等插件
也纳入了运行库清单。原供应商 OCCT 配置还需要 jemalloc；桌面 CI 的源码构建配置不需要它。

- FreeType 可选择 FreeType License（FTL）；需按其要求保留声明和鸣谢。
- FreeImage 使用 FreeImage Public License；内嵌 LibRaw 使用可选的 CDDL，JPEG、PNG、
  TIFF、OpenEXR、OpenJPEG、WebP、JXR、zlib 等组件保留各自声明，见 `licenses/runtime/FreeImage`。
  桌面 CI 从固定源码归档构建 FreeImage，包含本项目的 OpenEXR 兼容补丁；构建脚本、
  补丁和匹配 DLL 的构建记录随包提供。
- oneTBB 使用 Apache-2.0；jemalloc 以其 SDK 内的 BSD 声明为准。
- FFmpeg 桌面 CI 从固定的 3.3.4 源码构建，未启用 GPL 或 nonfree，采用 LGPL-2.1-or-later
  路径；替换 SDK 或构建选项后需要重新核对，不能把此结论套用到所有 FFmpeg 构建。
- OCCT 桌面 CI 从 7.9.3 源码构建，使用上游 NATIVE 内存分配器，链接源码构建的
  FreeImage、FFmpeg，以及清单中的 FreeType、oneTBB、OpenVR。
- Microsoft 运行库遵循微软再分发条款，不适用本项目 MIT。
- OpenVR 使用 BSD-3-Clause；DirectX Shader Compiler 同时涉及 LLVM/NCSA、MIT 和
  Microsoft 条款，副本见 `licenses/runtime/DirectXShaderCompiler`。

## 分发范围

MIT 允许使用、修改和再分发本项目自有代码，但不能替代第三方库要求的署名、许可副本、
对应源码提供方式，以及 LGPL 对库替换、重新链接和相关调试的要求。

Windows 打包会逐个检查实际部署的第三方 DLL 和微软运行库文件，生成
`share/licenses/OpenAutoPath/runtime-inventory.json`，记录组件、版本、SHA-256 和声明路径。
未知运行库、已审核 SDK 的二进制变化、缺失声明或源码归档哈希不符都会停止打包。
Qt 官方 SPDX 清单及构建配置也随包保存；许可集合含可选组件的声明，不表示这些组件全部启用。

包内 `share/sources/OpenAutoPath` 提供 Qt、OCCT、OpenCAMLib、FreeImage、FFmpeg、
FreeType 的固定版本源码归档和构建说明，详见
[依赖源码说明](licenses/DEPENDENCY_SOURCES.md)（安装后本文同目录也有该文件）。
已验证 48 个原供应商 DLL 与官方发布归档一致；这不等于已验证每个 DLL 的完整对应源码。
FreeImage、FFmpeg、OCCT 的桌面 CI 已配置本项目源码构建路径；带 `v` 版本号的打包
要求这三个组件具备匹配的源码、脚本和 DLL 构建记录。Qt 及其他预编译依赖的核对边界见依赖源码说明。
已发布的 ZIP 不会自动更新，第三方组件也不会改为 MIT。
