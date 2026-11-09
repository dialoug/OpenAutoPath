# 雕刻加工（OpenCAMLib）

`oap_toolpath_finishing` 提供 OpenCAMLib C++ drop-cutter 平行扫描雕刻，
对完整目标模型生成固定刀轴三轴候选刀路，可选择仅精雕或从项目毛坯分层开粗后精雕。
支持球头刀、平底刀和圆鼻刀。
它与现有外轮廓 Profile 工序独立，通过中立 `ToolpathProgram` 预览、验证和导出。

## 使用

1. 加载完整 STEP 模型并建立工件坐标，无需先识别孔、槽腔或外轮廓。
2. 在统一“刀路参数”的“雕刻参数”页设置方向、加工模式和刀具；默认六面分别生成固定刀轴刀路，也可单独选择 +Z、-Z、+X、-X、+Y 或 -Y。方向相对于项目工件坐标，可能与 STEP 世界坐标不同，应按装夹坐标选择浮雕正面。加工模式默认“仅精雕”，保持旧项目的贴面刀路语义；“从毛坯加工”显示分层最大切深和开粗保留余量。
3. 刀具可使用雕刻专用刀库（不改变其他工序的活动刀库），也可直接填写单刀刀型、刀号、尺寸、转速与进给。球头刀、圆鼻刀和平底刀均受支持。
4. 在 CAM 工作区添加雕刻，结果进入原有工序树、主视图、播放和参考 NC 导出流程；不再打开独立曲面参数/预览窗口。
5. 可先生成孔、槽腔或外轮廓，再生成雕刻。使用“加工项目 IPW”按全局顺序验证前序中间毛坯继续减料；已有中间件也可作为项目仿真毛坯。
6. 保存 .oapjob 后重开会重建刀路并恢复刀具与装夹；单面项目恢复对应方向，既有多面子集保留其装夹范围，不自动扩展为六面。修改雕刻参数后须重新生成。

刀库模式按球头、圆鼻、平底的顺序选择满足行距和最大刀径的已分配刀具，同类型优先较小直径。缺库或缺刀明确报错。单刀模式无需刀库，刀具快照直接绑定到正式工序。
雕刻默认行距 0.4 mm、沿行间距 0.25 mm、网格偏差 0.03 mm，可在统一参数页调整。

从毛坯加工使用正式项目毛坯：默认是各实体包络盒加项目 XYZ 余量，也支持整体模型包络盒和外部 STEP 毛坯。
外部毛坯 STEP 的内容指纹随项目保存，重开时变更会拒绝沿用旧结果。规划不读取仿真窗口的临时毛坯覆盖或 IPW 检查点。
在每个所选装夹坐标下，以保守毛坯包络限定开粗范围，从上至下分层，保护目标曲面并保留开粗余量，最后由同一把刀生成精雕刀路。
默认最大切深 1 mm、开粗保留余量 0.2 mm；开粗余量是沿刀轴加在精雕路径上方的额外余量，须不大于最大切深，最大切深须不大于刀具刃长。
最大切深控制相邻名义层高；每个采样点取该层高度与目标保护刀位的较高者，最后一粗层收尾到贴形留量。因此粗层可能沿曲面抬升，并非整行恒定 Z；这个层高限制不等于对行间残料实际吃刀量的认证。
各粗层与最终精雕共用预先验证的 XY 扫描分段；任一层无法通过自适应验证的间隙会在所有层保留，并通过抬刀连接。精雕时无接触区继续断开，不跨越间隙；相邻粗层覆盖检查仍保留，避免独立细分产生变化的断点而误报覆盖不足。模板计算同样受采样、工作量与取消检查约束。
仅精雕隐藏这两个开粗参数。旧偏好和旧项目缺少新字段时恢复为仅精雕，不会自动增加开粗工序。

两种模式都按独立扫描行并行计算接触点及自适应细分，每个工作线程独享刀具和
OpenCAMLib KD 树；结果按原行序合并，保留断点抬刀、粗层覆盖检查和稳定移动编号。
并行度同时受共享线程预算、采样规模与 KD 树复制量约束，小任务或预算不足时串行执行。
仅精雕的接触投影最多使用四个工作槽，避免 KD 树复制和查询争用抵消并行收益；
`OAP_CAM_MAX_THREADS` 可以进一步降低进程计算并行度。
多面目标的网格剖分也通过共享预算申请 OCCT 内部并行，剖分结束即释放线程配额，
随后才开始扫描行计算；OpenCAMLib 内部仍为单线程，避免嵌套线程池膨胀。
OCCT 后端不能确定内部线程上限，或无法取得完整线程配额时，剖分保持串行。
进度回调按完成行数串行发布，取消或采样预算耗尽时等待工作线程结束并丢弃全部候选刀路。

这是**按保守毛坯包络规划的分层开粗**，不会读取前序 IPW 的精确残料来裁剪扫描行或跳过空切。
先做孔槽等加工后再雕刻时，项目 IPW 可以按实际材料状态模拟整条工序链，但这不等于已实现残料自适应刀路优化。

全部完整模型实体参与投影，隐藏面也参与；每个独立接触段回到整个目标与规划毛坯最高点
上方的共同抬刀平面。参数变化会使旧结果失效，导出保留完整精度，预览在大结果
下抽样显示。余量定义为沿所选工件坐标 **+Z 的垂直偏置**，不是曲面法向等厚余量。
行距必须不大于刀径；减小行距、采样间距和离散偏差会增加计算量。

输出是采样刀刃接触路径。网格偏差、行距和采样间距会影响结果；局部补采样不能
证明任意连续曲面的完整无过切性。目前没有刀杆、刀柄、夹具、毛坯的完整碰撞认证，
正式 Job 与 IPW 已接通；仍不支持基于前序残料裁剪的余料精加工、等高 Waterline 或五轴加工。
参考 NC 仍须按项目现有后处理约定完成机床侧验证。

## 接口与边界

```cpp
oap::toolpath::SurfaceFinishingParams params;
params.cutterKind = oap::toolpath::FinishingCutterKind::Ball;
params.diameter = 6.0;
params.stepOver = 1.0;
auto result = oap::toolpath::SurfaceFinishing{}.generate(
    completeTargetShape, machiningFrame, params, control);
if (result.ok) {
    auto program = result.toProgram();
    auto programParams = result.toolpathParams();
    // 由调用方保留同一 machiningFrame 供显示和后处理使用。
}
```

- 公共接口仅依赖 core 与 OCCT，OpenCAMLib/Boost 不进入公共头文件。
- 私有 mesh 转换对模型深复制再网格化，保留源模型三角网缓存。
- 安全高度采用不依赖网格的目标与规划毛坯保守包围盒；复杂 B 样条使用控制点范围，避免耗时的精确极值求解。仅精雕扫描 XY 范围由加工网格决定，从毛坯加工使用毛坯包络范围。
- 刀位与移动以世界坐标保存，采样、余量与抬刀高度以 machiningFrame 为准。
- 拒绝非法或非有限参数，限制采样点、三角形和输出移动数量；默认 `maxMoves=2000000`，失败、取消不返回部分刀路。
  三角形上限在 OCCT 网格化完成后收集时检查，不是网格化阶段的硬内存上限；
  单次 OCCT 网格化与 OCL 投影调用内部不能立即中断。
- Qt 入口复用 `HeavyJobScheduler`，后台进度采用原子数据与现有进度窗口。
- 两种模式的扫描行通过 `ToolpathParallelLease` 借用进程统一预算并行投影；每个 worker 独立持有 OCL 刀具与 KD tree，从毛坯模式额外复用单行接触缓存，仅读取共享三角网。小网格自动串行，并限制复制索引的三角形总量；取不到工作槽立即串行回退。
- 各层复用模板内的接触高度，按固定行序组装移动并检查上一层覆盖；采样及校验预算跨 worker 共享，失败或取消会等待所有 worker 退出，不发布部分刀路。进度与取消回调串行调用，租约在组装前释放。
- 仅精雕与网格化暂保留串行路径；OpenCAMLib 内部 OpenMP 继续关闭，不另建线程池。
- `SurfaceFinishing` 是独立中立工序类型，不伪装成识别的 Pocket 特征。

## 正式工序与材料状态

`.oapjob` schema 12 使用 `surface_finishing_operations` 保存稳定 unit 引用、
参数、算法 ID、装夹引用、刀具控制器快照和全局顺序。旧的被移除的曲面工序字段
仍拒绝加载，避免把两种定义混用。运行时移动不写入项目文件。
新工序保存加工模式、分层切深、开粗余量和移动预算；仅精雕沿用 `opencamlib.surface.parallel`，
从毛坯加工使用 `opencamlib.engraving.stock-raster.v1`。旧项目迁移后仍为仅精雕。
编辑成功后保留工序 ID 与顺序；取消、非法刀型或生成失败不替换已有工序。
禁用工序保持定义但不播放或执行减料。刀库后续修改不静默改写已保存的刀具快照。

精加工与 Hole/Pocket/Profile 共用 IPW 父检查点。球刀、平底刀和圆鼻刀按真实
刀刃轮廓执行原生定轴减料，翻面沿用同一材料网格。修改参数、目标、刀具或装夹
会阻止使用旧结果，并使受影响的 IPW 后缀失效；未变化的前序检查点可复用。
IPW 减料结果不等同于完整工具装配碰撞认证。

孔、槽腔、外轮廓和雕刻均可在统一参数页直接填写单刀，也可选择各自支持的刀库来源。孔单刀支持钻头和平底铣刀；槽腔、外轮廓支持平底铣刀；雕刻支持球头、圆鼻和平底刀。单刀不恢复人工孔段或手动策略覆盖，仍使用各工序原有自动规划和校验流程。单刀无效时不会回退选库刀。
孔加工的现有自动策略仍将通孔交给钻削，平底盲孔和可铣沉孔肩部交给平底铣刀；单刀模式不会强制改变这一分工。
为兼容已有项目，内部类型 `SurfaceFinishing` 与 `surface_finishing_operations` 键保持不变。

## 构建依赖

[`cmake/OAPOpenCamLib.cmake`](../../../cmake/OAPOpenCamLib.cmake)
固定官方 OpenCAMLib revision `1a450b7c3d3b669224a2a5d09aab7f319794574c`，
构建可替换的 `oap_opencamlib` 动态库并关闭 OpenMP、Python 和 Node 绑定。
默认下载固定源码和经 SHA256 校验的 Boost 1.87.0 headers。
离线构建可设置 `OAP_OPENCAMLIB_ROOT` 和 `OAP_OPENCAMLIB_BOOST_ROOT`；
前者应指向指定 revision 的源码，后者应包含 `boost/version.hpp`。
运行目录的 `licenses/` 随库复制 OpenCAMLib LGPL 文本、来源说明和 Boost 许可证。

## 验证与测试模型

`surface_finishing` 回归测试直接链接真实 OpenCAMLib，检查三类刀具的解析斜面刀位、
球面球刀刀位、工件坐标变换、输入网格不变、程序验证与参考 NC 导出，
以及非法参数、资源预算和取消的事务性输出。
分层回归还覆盖移动层高折点、往返扫描行的固定断点、翻转装夹、模板计算阶段的取消和采样预算拒绝。可单独运行 `oap_test_surface_finishing --stable-layer-template`。

构建后运行 `ctest --test-dir build -C RelWithDebInfo -R surface_finishing --output-on-failure`。
测试可执行文件支持 `--write-samples <目录>`，生成斜面、球体、平面块和分离岛屿
STEP 模型用于交互验收；运行时 PATH 需包含 OCCT 和应用 bin 目录。

2026-11-08 移除人工工艺与手工刀具后，本机 RelWithDebInfo 主程序构建成功，CTest **60/60** 通过。
该记录对应雕刻统一流程改造之前的版本；独立曲面窗口现已移除。
日志为 `build/remove-manual-final-build.log` 与 `build/remove-manual-ctest.log`；
已生成的四个 STEP 样本和窗口截图位于 `artifacts/finishing/`（本地验收产物，不入库）。

新增集成测试 `surface_finishing_job`、`surface_finishing_ipw`、
`surface_finishing_project` 覆盖持久化迁移、粗→精材料链、桌面工作区重载、
工序树、参考导出、仿真毛坯与规划毛坯隔离，以及禁用精加工项目重开。
下一阶段可补完整工具装配与连续运动校验，以及 Waterline、
局部加工边界、基于残料的裁剪与自动残高控制。

雕刻回归新增 `engraving_workflow` 与 `engraving_relief`，覆盖默认六面、独立 +Z、直接单刀、专用刀库、失败/取消原子性和粗→雕刻 IPW。统一参数页与偏好往返由 `toolpath_parameter_dialog` 验证。
实际浮雕验收可运行 `oap_test_engraving_relief --step <STEP路径> --output <输出目录>`；支持刀径、行距、采样、网格偏差与超时参数。真实客户模型仅在本机使用，不入库。
复现桌面选刀与装夹映射可运行 `oap_test_engraving_workflow --step <STEP路径> --library resources/tool-libraries/general-simulation.oaptl --scope -Z`；该诊断模式使用正式 STEP 索引、工件坐标与雕刻工作流，打印实际刀具和坐标轴。它与 `engraving_relief` 的世界坐标方向参数含义不同。

2026-11-09 分层覆盖回归：`rounded_relief.step` 使用模拟刀库自动选中的 D0.5 平底刀、桌面 `-Z` 装夹、零毛坯余量及默认雕刻参数，旧实现复现前层覆盖不足错误，共享扫描模板修复后生成成功：8 层粗加工加最终精雕，共 1,336,265 段移动，低于默认 2,000,000 上限。该模型的工件 `-Z` 实际映射世界 `+Z`。小型斜面用例同样验证旧实现失败、新实现通过；完整应用构建成功，CTest **62/62** 通过。日志为 `build/engraving-coverage-desktop-baseline.log`、`build/engraving-coverage-desktop-fixed.log`、`build/engraving-coverage-final-build.log` 与 `build/engraving-coverage-final-tests.log`。

毛坯模式验收增加 `--mode stock --stock-margin 1 --step-down 1 --roughing-allowance 0.2`；
`--stock-margin` 在此验收工具中构造目标包围盒外扩的测试毛坯，正式应用使用项目毛坯配置。
2026-11-09，本机 `rounded_relief.step` 的 +Z 毛坯模式通过生成及参考 NC 检查：D2 球刀、
行距 0.8、采样间距 0.5、网格偏差 0.05 mm，10 层开粗加最终精雕，435,918 段移动
（其中开粗 396,883 段）。输出位于 `artifacts/engraving/rounded-relief-stock/`，
日志为 `build/engraving-stock-rounded-relief.log`。该验收验证路径和参考输出，不是机床实切认证。

2026-11-08 雕刻统一流程改造：RelWithDebInfo 完整应用构建成功，CTest **61/61** 通过；日志为 `build/engraving-full-build.log` 与 `build/engraving-full-ctest.log`。

本机 `rounded_relief.step`（135 面、1 实体）+Z 验收：D2 球刀、行距 0.4、采样 0.25、偏差 0.03 mm，181,298 三角形、435,601 采样、301 行、147,816 段移动。保守包围盒优化前后全部 146,725 段切削移动逐行一致，仅安全抬刀平面约增加 0.0022 mm。参考 NC 与 CSV 在 `artifacts/engraving/rounded-relief/`，对比日志为 `build/engraving-rounded-relief-comparison.log`。真实样件验证 +Z；六面及粗→雕刻链由合成工作流测试覆盖。实件与全量编译并行运行，墙钟耗时不作为严格性能对照。
