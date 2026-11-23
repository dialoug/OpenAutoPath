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
| [nlohmann/json](https://github.com/nlohmann/json/releases/tag/v3.12.0) | 3.12.0 | C++ JSON 读写；MIT，见 [许可文本](licenses/nlohmann-json-MIT.txt) |
| React、React DOM、Scheduler、Three.js、Lucide | 见 Web 锁文件 | 浏览器依赖；构建产物含 third-party-licenses.txt |

安装的 ServerWorker 和 Development 组件包含项目许可与原生依赖声明。Windows worker 的 OCCT 运行库可能依赖 FreeType 2.13.3、FreeImage 3.18.0、oneTBB 2021.13.0、OpenVR 1.14.15、jemalloc、FFmpeg 3.3.4 及 Microsoft Visual C++ 运行库。实际文件由 CMake 依照 worker 的 DLL 依赖解析。原始许可证和对应源码信息见 [运行库许可](licenses/runtime/README.md) 和 [依赖源码](licenses/DEPENDENCY_SOURCES.md)。

Windows 官方 SDK 包含 FFmpeg 依赖。可用仓库的源码构建脚本生成关闭 FFmpeg 的 OCCT，供使用该配置的部署使用。

OCCT 与 OpenCAMLib 以共享库形式构建和加载。分发时按上游条款保留许可、署名及对应源码获取方式。
