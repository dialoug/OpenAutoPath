# 浏览器 CAM 工作台

本工作台复用 OpenAutoPath 的 STEP 分析、加工项目、刀库、刀路规划、余料仿真和参考 NC 后处理。浏览器负责特征选择和三维显示，Go 负责上传、会话和任务，一个无界面 C++ worker 进程执行一次任务。可用仓库的 [CAM 测试零件](../samples/cam_feature_test.stp) 体验完整流程。

## 在 Windows 构建

从仓库根目录运行以下 PowerShell 命令。先按[主项目构建说明](../BUILDING.md)配置 C++ 和 OCCT，再准备 Go 1.23 或更高版本，以及带 npm 的 Node.js 22。VS 2026 用户可将以下 `windows-core` 替换为 `windows-core-vs2026`。

```powershell
cmake --preset windows-core
cmake --build --preset windows-core --target oap-worker
cmake --install build/windows-core --config Release --component ServerWorker --prefix build/cam-server-package

go -C server build -o ../build/cam-server-package/oap-server.exe ./cmd/oap-server
Push-Location web
npm ci
npm run build -- --outDir ../build/cam-server-package/web
Pop-Location
```

`ServerWorker` 安装组件将 worker、Windows 原生依赖和许可文件放在同一个目录中。部署机需要项目对应的 Microsoft Visual C++ x64 运行库。网页构建产物由 Go 直接提供，运行服务器时只需安装包和可写的数据目录。

## 启动与使用

```powershell
Push-Location build/cam-server-package
.\oap-server.exe -listen 127.0.0.1:8080 -worker bin/oap-worker.exe -web web -data ../cam-data
```

本机后续更新统一使用 8080。等正在执行的任务结束，停止该实例，更新 `build/cam-server-package` 中的服务、worker 和网页，再用上面的命令启动，沿用 `build/cam-data` 中的会话与任务记录。

首次建立 STEP 索引时，worker 可并行计算复杂曲面的精确包围盒。默认线程上限为本机逻辑处理器数。要限制单个 worker 的线程数，可在启动服务前设置环境变量 `OAP_CAM_MAX_THREADS`，例如在 PowerShell 中执行 `$env:OAP_CAM_MAX_THREADS = "4"`。同一模型后续任务优先复用已校验的包围盒缓存。

在浏览器打开 `http://127.0.0.1:8080`。上传 STEP 后，点击孔壁或从列表选孔，选择模拟刀库中的刀具，填写进给和转速，再点击“生成参考刀路”。例如，测试件中的 Ø6 mm 通孔可选直径 6 mm 的钻头。完成后可以回放刀路、查看本次验证结果和下载参考 NC。网页标明本次导出的孔与刀具，以及孔覆盖和实体扫掠审计状态。

在右侧“加工特征”中切换到“槽腔加工”，选择区域、铣刀、策略和层深即可追加工序。已有历史模型缺少区域数据时，服务会使用原 STEP 补识别。首次规划自动创建项目，后续规划保存到当前模型所选项目的新版本。

### 加工项目与刀库

1. 在“刀库管理”中从内置模拟刀库创建版本，或导入已有的 `.oaptl`。选择刀具后可以编辑类型、几何、T 号、进给和转速。保存后点击“用于加工”，工作台会使用这个版本。
2. 上传 STEP 或切换到会话内的历史模型。在“加工项目”中新建项目或导入与当前 STEP 内容指纹匹配的 `.oapjob`。可以从已有孔加工结果加入工序，也可以根据识别区域或装夹方向规划槽腔、外轮廓和雕刻。每次规划槽腔会追加一组工序，分别保留区域、刀具、策略和层深。外轮廓仍按该类工序更新，旧版本可在列表中打开。
3. 调整毛坯余量、工序顺序、启用状态和项目刀具快照，保存为新版本。“重算并预览”计算所选工序的刀路与参考 NC。“仿真已启用工序”按项目顺序计算余料，可切换查看最终余料和零件。
4. 在槽腔工序下点击“更换刀具”，从当前活动刀库搜索铣刀，再选“换刀并重算此工序”。成功后可预览刀路并下载该版本的参考 NC；计算失败时可回到原版本继续编辑。测试件的两个槽腔可分别选 D6 偏置、层深 2 mm 和 D4 往复、层深 1 mm，再把第二道换为 D3。
5. 刀库支持按类型、名称、刀位或直径查找，也可从其他已保存的刀库版本取用刀具，编辑钻削循环、啄钻深度、冷却与快移。跨库取用会复制参数到当前草稿，保存后生效。
6. 带独立槽腔参数的项目使用 V13，其余项目继续输出 V12。可导入与当前 STEP 匹配的历史项目。外部 STEP 毛坯可从当前会话中已上传的模型绑定，外部碰撞模型还没有 Web 上传入口。

服务默认监听本机。可信局域网演示可通过 `-listen 192.168.1.10:8080` 指定服务器实际网卡地址。服务以浏览器会话隔离文件，任务依赖同一会话 cookie 继续访问。面向公网的用户登录、项目共享和配额管理属于后续工作。

### 余料回放

“仿真与回放精度”提供交互、平衡、详细三档，浏览器初始选择详细档。仿真按项目启用刀具的最小直径和预设的材料网格预算选取间距，显示网格另按同档预算生成。回放进度栏显示当前帧实际的显示间距。打开已有回放后，可以在加工结果中选择精度并重算，播放位置会保留；复核某一步会沿用这次回放的材料精度。浏览器按帧读取网格，并预取下一帧。

在“加工项目 → 仿真与 NC”选择“回放已启用工序”，或在工序列表选择“回放此工序余料”。单道工序从前序工序留下的余料开始。已生成的刀路或静态余料也可从“加工结果”进入回放。

回放支持倍速、暂停、逐步、跳至工序、显示刀具和跟随刀尖。服务端用原生 Multi-Dexel 内核生成最多 33 帧余料，浏览器在各移动位置显示最近一帧的序号和体积。点击“查看此步余料”可对当前移动重新计算并查看余量和过切，点击“返回上次结果”可回到刚才的位置。

`-workers` 设置同时运行的原生进程数，默认 1。每次上传对应一个独立模型版本，单文件上限为 128 MiB。关闭页面后任务继续执行，重新打开工作台可恢复最近一次任务。服务重启后可以读取已完成的结果，中断的任务可重新提交。同一数据目录由一个 Go 服务实例管理。

## 接口与原生回归

HTTP 消息和状态码可查阅 [OpenAPI 定义](../contracts/http/openapi.yaml)。孔加工请求样例使用测试件中的 F10 孔和模拟刀库中的 D6 钻头。[worker 加工请求](../contracts/worker/generate.json)

| 操作 | HTTP |
|---|---|
| 建立浏览器会话 | `GET /api/session` |
| 上传模型并排队分析 | `POST /api/models`，正文为 STEP 原始文件 |
| 计算孔刀路 | `POST /api/models/{modelId}/toolpaths`，JSON 字段为 `hole`、`tool`、`feed`、`rpm` |
| 列出会话的模型及文档版本 | `GET /api/workspace` |
| 新建、导入或保存刀库 | `POST /api/libraries` |
| 新建、导入或保存项目 | `POST /api/models/{modelId}/projects` |
| 规划工序、重算或仿真 | 在项目 URL 后追加 `/plan`、`/rebuild` 或 `/simulate` 并 POST |
| 按活动刀库重新规划所有孔 | 项目 URL 后追加 `/holes`，传 `document` 和可选 `libraryId` |
| 绑定外部毛坯 | 项目 URL 后追加 `/stock`，传 `document` 和会话内已上传模型的 `stockId` |
| 核对装夹与导出项目 NC | 先 POST `/setups` 保存 `confirmedSetups`，再用该版本的 `document` POST `/export`。审查报告为 `programs.json`，所有程序通过结构审查后提供 `programs.zip` |
| 指定余料阶段 | POST `/simulate` 可传终点工序 `operation` 和该工序内可选的 `completedMoves`，传 0 表示开始该工序前 |
| 准备余料回放 | POST `/replay` 传 `document`，可选 `operation` 限定回放工序。产物包括刀路、工序和余料帧索引、二进制显示网格 |
| 查询或取消任务 | `GET` 或 `DELETE /api/jobs/{jobId}` |
| 获取已完成的结果 | `GET /api/jobs/{jobId}/files/{name}` |

Go 为 worker 写入 v1 请求文件，并将当前工作目录设为本次任务目录。worker 输出 JSON 行进度事件，成功时生成 `result.json`。分析结果包含网格和刀库；项目、规划和仿真任务保存原生文档。成功任务的 `files` 列出可下载产物，仿真结果的 `volume` 单位为 mm³。任务目录中的 `request.json` 保留确切输入，便于在独立工作目录重放。模型、刀具和协议约定见[请求样例](../contracts/worker/analyze.json)。

在工作台的「加工结果」中选择「审查本次参考 NC」可以查看单道工序。项目完整导出位于「加工项目 → 仿真与 NC」。审查报告记录每个文件的装夹、刀具、工序、NC 原文及 C++ 内核给出的结构错误和警告。问题行号从 1 开始，0 表示问题涉及整个文件。页面支持点击问题定位、搜索和跳行。逐一核对各文件后，页面提供整包下载。同一浏览器标签页会记住当前任务的核对进度。

在仓库根目录运行真实 worker 的集成测试。测试通过 HTTP 上传损坏文件和真实模型，检查识别面与网格映射、生成刀路、下载 NC，并验证跨会话访问隔离。

```powershell
$env:OAP_TEST_WORKER = (Resolve-Path build/cam-server-package/bin/oap-worker.exe).Path
go -C server test -count=1 -v ./...
```
