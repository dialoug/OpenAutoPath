# 加工测试样例

这些样例用于手动验证模型导入、特征识别、刀路生成和余料仿真。
它们不属于默认 CTest 自动回归输入。

| 文件 | 用途 |
| --- | --- |
| [cam_feature_test.stp](cam_feature_test.stp) | 包含外轮廓、圆角矩形腔、长圆槽、通孔、盲孔和沉孔的组合零件，可用于孔、槽腔和外轮廓加工测试 |
| [generate_cam_feature_test.cpp](generate_cam_feature_test.cpp) | 组合零件的 C++20/OCCT 生成器，包含实体有效性、解析体积及 STEP 导出后重新导入的校验 |
| [rounded_relief.step](rounded_relief.step) | 曲面测试模型，可用于曲面雕刻、加工参数调整和余料仿真；文件约 23.4 MiB |

## 使用方法

1. 克隆仓库取得样例，或在 GitHub 对应文件页下载原始文件。
2. 在 OpenAutoPath 中打开 `.stp` 或 `.step` 文件。
3. 确认模型尺寸，设置毛坯、工件坐标和装夹方向，选择刀具及加工参数。
4. 组合零件先进行特征识别，再配置孔、槽腔或外轮廓工序；曲面模型可进入雕刻加工流程。
5. 生成刀路后检查路径与余料仿真，按需保存加工项目并导出参考 NC。

这些文件只包含模型或生成源码，不包含预设的机床、刀具和加工项目。
更多操作说明见[项目 README](../README.md)及[雕刻加工说明](../src/toolpath/finishing/README.md)。

## 生成器说明

`generate_cam_feature_test.cpp` 当前未加入默认 CMake 构建目标。
使用 C++20 和项目所用的 OCCT 开发环境编译后，运行时传入一个输出 STEP 路径。
程序会检查生成实体及导出后重新导入的模型；校验成功时输出 `VALIDATED` 信息。
生成模型使用毫米单位。
