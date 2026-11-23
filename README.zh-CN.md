# OpenAutoPath

[English](README.md) · [构建与启动](server/README.md) · [架构说明](ARCHITECTURE.md)

OpenAutoPath 是浏览器 CAM 工作台。上传 STEP 模型后，可查看识别出的特征，规划孔、槽腔、外轮廓和雕刻工序，管理加工项目与刀库，在三维视图中预览刀路和余料，并审查和下载参考 NC。

Go HTTP 服务托管网页并管理模型、项目、会话与计算任务。几何分析、刀路和余料计算由独立的 C++ worker 执行，浏览器使用 React 和 Three.js。

- [构建依赖与原生测试](BUILDING.md)
- [加工样件](samples/README.md)
- [HTTP 接口](contracts/http/openapi.yaml)
- [项目许可证](LICENSE)和[第三方许可](THIRD_PARTY_NOTICES.md)
