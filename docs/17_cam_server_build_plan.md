# CAM Web 服务

当前项目使用 React / Three.js 工作台、Go HTTP 服务和 C++ worker。Go 服务按会话管理 STEP 文件、刀库、项目与计算任务。C++ worker 复用领域模块，执行 STEP 分析、孔与槽腔加工、外轮廓、雕刻和余料仿真；浏览器提供模型、刀路及回放视图。

每个原生任务使用独立目录和版本化请求。标准输出承载逐行 JSON 进度，完成后发布结果清单。网格、回放和 NC 为单独文件。完成与取消状态由 Go 管理，浏览器轮询任务接口。

- [构建、启动和验收步骤](../server/README.md)
- [模块和部署架构](../ARCHITECTURE.md)
- [HTTP 接口](../contracts/http/openapi.yaml)和 [worker 请求](../contracts/worker/analyze.json)
