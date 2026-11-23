# OpenAutoPath 架构

```text
浏览器（React / Three.js）
  │ HTTP：模型、项目、刀库、任务和产物
  ▼
Go 服务（oap-server）
  │ 版本化 JSON 请求、进度事件、结果文件
  ▼
C++ worker（oap-worker）
  ├─ analysis / recognition：STEP 分析与特征识别
  ├─ tooling：刀具资产、刀库格式和加工参数适配
  ├─ toolpath：刀路算法、验证和 NC 后处理
  └─ manufacturing：加工项目、工序和 Multi-Dexel 余料
```

Go 服务向浏览器提供静态资源和 API，按会话隔离文件和任务。每次计算启动一个 worker 进程，读入独立目录中的任务参数，通过标准输出报告进度，在成功时保存结果。浏览器负责交互和三维显示。部署时由 Go 服务同时提供网页和 API。

- [构建、运行和原生集成测试](server/README.md)
- [HTTP 契约](contracts/http/openapi.yaml)与 [worker 请求样例](contracts/worker/analyze.json)
- [特征识别](src/recognition/README.md)、[刀路](src/toolpath/README.md)和[加工项目](src/manufacturing/README.md)
- [C++ 模块依赖](cmake/OAPModuleDependencies.cmake)和[公开头文件清单](cmake/OAPPublicHeaders.cmake)

C++ 构建和架构检查共用目标清单及直接依赖声明。跨目标调用遵守公开头文件清单。C++ worker 内的 JSON 及文件读写属于服务协议和领域存储，浏览器和 Go 服务器保持现有文档格式，项目与刀库仍使用 `.oapjob`、`.oaptl`。
