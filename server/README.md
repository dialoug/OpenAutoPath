# 浏览器 CAM 工作台

本工作台复用 OpenAutoPath 的 STEP 分析、加工项目、刀库、刀路规划、余料仿真和参考 NC 后处理。浏览器负责特征选择和三维显示，Go 负责上传、会话和任务，一个无界面 C++ worker 进程执行一次任务。可用仓库的 [CAM 测试零件](../samples/cam_feature_test.stp) 体验完整流程。

## 在 Windows 构建

从仓库根目录运行以下 PowerShell 命令。先按[主项目构建说明](../BUILDING.md)配置 C++、Qt 和 OCCT，再准备 Go 1.23 或更高版本，以及带 npm 的 Node.js 22。VS 2026 用户可将以下 `windows-core` 替换为 `windows-core-vs2026`。

```powershell
cmake --preset windows-core -DOAP_BUILD_WORKER=ON
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

在浏览器打开 `http://127.0.0.1:8080`。上传 STEP 后，点击孔壁或从列表选孔，选择模拟刀库中的刀具，填写进给和转速，再点击“生成参考刀路”。例如，测试件中的 Ø6 mm 通孔可选直径 6 mm 的钻头。完成后可以回放刀路、查看本次验证结果和下载参考 NC。网页标明本次导出的孔与刀具，以及孔覆盖和实体扫掠审计状态。

### 加工项目与刀库

1. 在“刀库管理”中从内置模拟刀库创建版本，或导入桌面版的 `.oaptl`。选择刀具后可以编辑类型、几何、T 号、进给和转速。保存后点击“用于加工”，工作台会使用这个版本。
2. 上传 STEP 或切换到会话内的历史模型。在“加工项目”中新建项目或导入与当前 STEP 内容指纹匹配的 `.oapjob`。可以从已有孔加工结果加入工序，也可以根据识别区域或装夹方向规划槽腔、外轮廓和雕刻。规划槽腔或外轮廓会更新项目中该类工序，旧版本仍可在列表中打开。
3. 调整毛坯余量、工序顺序、启用状态和项目刀具快照，保存为新版本。“重算并预览”计算所选工序的刀路与参考 NC。“仿真已启用工序”按项目顺序计算余料，可切换查看最终余料和零件。
4. 下载项目或刀库后可用桌面版的对应模块读取。项目引用的外部 STEP 毛坯与碰撞模型目前只能在桌面端参与计算。浏览器会话拥有自己的版本历史。

服务默认监听本机。可信局域网演示可通过 `-listen 192.168.1.10:8080` 指定服务器实际网卡地址。服务以浏览器会话隔离文件，任务依赖同一会话 cookie 继续访问。面向公网的用户登录、项目共享和配额管理属于后续工作。

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
| 查询或取消任务 | `GET` 或 `DELETE /api/jobs/{jobId}` |
| 获取已完成的结果 | `GET /api/jobs/{jobId}/files/{name}` |

Go 为 worker 写入 v1 请求文件，并将当前工作目录设为本次任务目录。worker 输出 JSON 行进度事件，成功时生成 `result.json`。分析结果包含网格和刀库；项目、规划和仿真任务保存原生文档。成功任务的 `files` 列出可下载产物，仿真结果的 `volume` 单位为 mm³。任务目录中的 `request.json` 保留确切输入，便于在独立工作目录重放。模型、刀具和协议约定见[请求样例](../contracts/worker/analyze.json)。

在仓库根目录运行真实 worker 的集成测试。测试通过 HTTP 上传损坏文件和真实模型，检查识别面与网格映射、生成刀路、下载 NC，并验证跨会话访问隔离。

```powershell
$env:OAP_TEST_WORKER = (Resolve-Path build/cam-server-package/bin/oap-worker.exe).Path
go -C server test -count=1 -v ./...
```
