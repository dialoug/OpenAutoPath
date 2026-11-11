# OpenAutoPath

[English](README.md) | **简体中文**

OpenAutoPath 是一个基于 **C++20、Qt 和 Open CASCADE（OCCT）** 的 CAD/CAM 桌面项目，
用于查看 STEP 模型、识别加工特征、规划刀路、模拟材料去除，并导出参考 NC 程序。

项目目前提供 **Windows x64 桌面版**。Linux 和 macOS 已接入核心模块的构建与测试，
尚未提供桌面应用发布包。

[下载 Windows 版](https://github.com/dialoug/OpenAutoPath/releases/latest) ·
[构建说明](BUILDING.md) · [项目架构](ARCHITECTURE.md) ·
[CI 与发布记录](https://github.com/dialoug/OpenAutoPath/actions) ·
[问题反馈](https://github.com/dialoug/OpenAutoPath/issues)

界面支持简体中文（默认）和英语。在 **视图 → 语言 → 英语** 中切换，重新启动后生效；英文界面可通过 **View → Language → Simplified Chinese** 切回中文。语言选择会保存在本机，翻译资源已内置，无需单独安装。

## 可以做什么

| 功能 | 当前能力 |
| --- | --- |
| 模型查看与分析 | 导入 STEP 模型，查看几何与拓扑结构，进行选择和分析 |
| 加工特征识别 | 识别孔、槽腔和外轮廓相关加工区域，供后续规划使用 |
| 孔加工 | 生成钻削及孔铣削等固定刀轴刀路 |
| 槽腔加工 | 生成偏置、往复、螺旋及 FreePath 清料路径，安排进刀和连接 |
| 外轮廓加工 | 生成固定刀轴外部清料与轮廓精加工路径 |
| 曲面雕刻 | 使用 OpenCAMLib 进行固定刀轴往复扫描，支持球头、平底和圆鼻刀；可仅精雕或从毛坯分层开粗后精雕 |
| 加工项目 | 保存工序、装夹、刀具与参数，支持项目重建和按工序组织刀路 |
| 余料仿真 | 按工序顺序计算中间毛坯（IPW），显示材料去除过程并回放刀路 |
| NC 导出 | 通过项目后处理器导出参考 NC，按路径原语表达直线、适用的圆弧及钻削循环 |

加工项目使用 `.oapjob` 文件，刀库使用 `.oaptl` 文件。

## 下载与启动

1. 打开 [Releases](https://github.com/dialoug/OpenAutoPath/releases)，选择需要的版本。
2. 下载附件中名称为 `OpenAutoPath-版本号-windows-x64.zip` 的程序包。
   GitHub 自动提供的 `Source code` 压缩包是源码，不是可运行程序。
3. 完整解压，保留 `bin`、`share` 等目录及其相对位置。
4. 运行解压目录根部的 `OpenAutoPath.exe`。
5. 如果系统缺少 Visual C++ 运行库，先运行包内的 `bin/vc_redist.x64.exe`。

发布包附带 `SHA256SUMS.txt`，可用于核对下载完整性。包内的 `build-info.json`
记录版本、构建配置和源码提交；`README.txt` 提供中英文启动说明。
当前 ZIP 未进行代码签名。

## 第一次使用

建议先选择结构简单、尺寸明确的 STEP 模型，走通以下流程：

1. **打开模型**：导入 `.step` 或 `.stp` 文件，检查显示和模型尺寸。
2. **确定加工条件**：设置毛坯、工件坐标与装夹方向，并选择刀具及加工参数。
3. **选择加工方式**：孔、槽腔和外轮廓加工先进行相应特征识别；曲面雕刻可以直接以完整目标模型进行规划。
4. **生成刀路**：在 CAM 工作区添加对应工序，检查生成结果、进退刀和路径连接。
5. **查看余料**：使用加工项目 IPW 检查工序顺序下的材料去除结果，回放刀路。
6. **保存与导出**：保存加工项目；需要 NC 时，检查装夹、刀具和程序审查结果后导出参考 NC。

曲面雕刻的具体参数和操作入口见 [雕刻加工说明](src/toolpath/finishing/README.md)。
仓库提供[测试样例及使用说明](samples/README.md)，可下载其中的 STEP 模型，
用于尝试特征识别、孔/槽腔/轮廓加工及曲面雕刻流程。

## 当前边界

- **平台**：Windows 支持桌面应用；Linux/macOS 当前只验证核心模块与安装后的 SDK。
- **曲面加工**：当前集成的是固定刀轴扫描，不是五轴联动加工；尚未接入等高 Waterline 刀路。
- **余料利用**：曲面雕刻的分层开粗依据毛坯包络规划，尚未按前序 IPW 的精确残料裁剪扫描路径。
- **仿真与 NC**：结果受网格、采样和仿真分辨率影响；当前未提供刀杆、刀柄、夹具等完整碰撞认证。
  NC 输出是参考后处理结果，实际加工前需要针对目标机床和控制器验证。
- **开发接口**：核心库可安装并通过 CMake 使用，但尚未承诺公共 API 或 ABI 的版本稳定性。

## 从源码构建

需要 Git、CMake 3.22 或更新版本，以及支持 C++20 的编译器。
当前依赖基线为 Qt 6.11.1 和 OCCT 7.9.3；核心构建也需要 Qt Core 等依赖。

### Windows 桌面版

桌面版需要按[依赖源码说明](licenses/DEPENDENCY_SOURCES.md)构建 FreeImage 和关闭 FFmpeg 的 OCCT；原预编译 OCCT SDK 可用于核心测试。

先安装 Visual Studio 2022 的 C++ 桌面开发工具和 Qt 6.11.1 的
`msvc2022_64` 组件（含 Qt Shader Tools），然后在 PowerShell 中执行：

```powershell
git clone https://github.com/dialoug/OpenAutoPath.git
cd OpenAutoPath

# 下载并校验 OCCT SDK；此脚本不安装 Qt。
powershell -NoProfile -ExecutionPolicy Bypass -File tools/Install-WindowsOcct.ps1

# 先按下文链接的“依赖源码说明”构建 FreeImage 和不含视频支持的 OCCT。
# Qt 安装在其他位置时，替换下面的路径。
cmake --preset windows-vs2022 -DOAP_QT_ROOT=C:/Qt/6.11.1/msvc2022_64 -DOAP_FREEIMAGE_ROOT="$PWD/build/dependencies/freeimage" -DOAP_OCCT_ROOT="$PWD/build/dependencies/occt/install"
cmake --build --preset windows-vs2022
ctest --preset windows-vs2022
ctest --preset windows-vs2022-packaging

./build/windows-vs2022/RelWithDebInfo/OpenAutoPath.exe
```

使用 Visual Studio 2026 时，将预设名替换为 `windows-vs2026`，启动路径中的目录名
也相应替换。首次配置可能需要联网获取固定版本的其他依赖。
随项目使用的 Windows OCCT SDK 只提供发布配置，桌面预设使用 `RelWithDebInfo`，
不支持直接切换到 Debug。

### Linux / macOS 核心模块

分别使用 `linux-core` 和 `macos-core` 预设。依赖安装、与 CI 一致的 micromamba 环境、
Qt/OCCT 路径设置和安装方法见 [BUILDING.md](BUILDING.md)。

## 工程与验证

| 目录或文件 | 内容 |
| --- | --- |
| `src/app/` | Qt 桌面界面与工作流协调 |
| `src/analysis/`、`src/recognition/` | 几何分析与特征识别 |
| `src/toolpath/` | 刀路规划、验证、后处理与材料去除内核 |
| `src/manufacturing/` | 加工项目、工序和 IPW 生命周期 |
| `src/tooling/` | 刀具库与加工参数适配 |
| `tests/` | 各模块的自动化测试 |
| `cmake/`、`CMakePresets.json` | 构建目标、依赖、测试与安装规则 |
| `.github/workflows/` | CI 和版本发布流程 |

分支推送和 Pull Request 会触发架构检查及 Windows/Linux/macOS 构建测试。
版本标签触发 Release 流程：所有检查通过后发布 Windows 包；手动运行 Release 仅生成
可下载的构建产物，不创建正式发布。详细规则见 [Windows 自动发布说明](BUILDING.md#windows-release-delivery--windows-自动打包与发布)。

进一步阅读：[架构文档](ARCHITECTURE.md)、[刀路模块说明](src/toolpath/README.md)、
[加工项目说明](src/manufacturing/README.md)。

## 问题反馈与参与

请通过 [GitHub Issues](https://github.com/dialoug/OpenAutoPath/issues) 报告问题。
建议提供软件版本、操作系统、复现步骤、预期与实际结果，以及必要的日志或截图。
几何或刀路问题尽量附上可公开分享的最小模型和参数；不要上传无权分享的模型或业务数据。

提交代码修改前，请阅读构建与架构文档，运行对应模块测试，并在 Pull Request 中说明
修改原因和验证结果。独立的贡献指南与反馈模板尚待补充。

### README 多语言维护

`README.md` 是英文首页，`README.zh-CN.md` 是简体中文版，两者均在仓库根目录维护。
修改功能、限制、依赖版本或构建命令时，请在同一个 Pull Request 中同步更新两个版本，
保持章节结构和技术信息一致；文件名、命令、参数及链接目标不翻译。
详细构建和架构说明继续集中维护在 `BUILDING.md`、`ARCHITECTURE.md`，不在 README 中重复展开。

## 许可证

项目自有代码和文档采用 [MIT 许可证](LICENSE)，版权署名为 `2026 dialoug`。
第三方组件及移植代码保留原许可；其中二面角分析的 AnalysisSitus 移植部分采用
BSD-3-Clause。依赖许可、安装包声明范围和再分发要求见
[第三方组件与许可说明](THIRD_PARTY_NOTICES.md)。
