# 刀路算法注册层开发规划

> 文档状态：执行中
>
> 当前主阶段：A2——算法元数据、能力和版本治理
>
> 当前任务键：`A2/registry-metadata`
>
> 代码审计基线：`612ab76`
>
> 基线日期：2026-08-24
>
> 最近验证：2026-08-24，Release CTest 51/51 通过
>
> 维护角色：`oap_toolpath_algorithms` 目标维护者

本文维护算法接口、内置实现适配和不可变注册表。具体几何/安全语义由 [Hole](../hole/DEVELOPMENT_PLAN.md)、[Pocket](../pocket/DEVELOPMENT_PLAN.md)、[Profile](../profile/DEVELOPMENT_PLAN.md) 计划负责；本层不复制其算法路线图。

## 当前基线

- `IPocketToolpathAlgorithm`、`IHoleToolpathAlgorithm`、`IProfileToolpathAlgorithm` 已形成独立接口；
- 内置 Hole/Pocket/Profile 算法使用稳定字符串 ID；
- `ToolpathAlgorithmRegistry` 可注入、无全局可变状态，并以只读共享对象供后台任务使用；
- `ToolpathAlgorithmSelection` 明确每个领域所选算法；
- 默认注册表只装入当前可用的内置实现。

## 不可破坏的契约

1. 算法对象无共享可变状态，可被多个后台任务并发读取。
2. 已公开 ID 不改变含义；行为模型不兼容时注册新 ID/版本。
3. 查找失败返回明确 unavailable，不回退到另一工序家族或语义不同算法。
4. 内置适配器必须执行对应特征模块的完整验证门禁，不能只包装候选路径。
5. 默认注册表是生产准入列表，不是所有实验实现的集合。
6. 第三方算法的取消兼容路径必须在元数据中显式说明，不能宣称未实现的能力。

## 路线图

| 阶段 | 优先级 | 状态 | 交付 |
| --- | --- | --- | --- |
| A0 三领域接口与注册表 | P0 | 完成 | 稳定 ID、注册/查询、选择和内置实现 |
| A1 异步只读注入 | P0 | 完成 | 不可变共享注册表和协作取消入口 |
| A2 元数据与版本治理 | P0 | **下一阶段** | 能力、模型版本、刀具/Setup 支持、正式准入状态 |
| A3 外部算法加载边界 | P2 | 待实施 | 版本协商、隔离、诊断和可选插件适配 |

## A2 实施重点

- 为每个算法发布稳定元数据：领域、ID、实现版本、验证模型 ID、支持刀具/固定轴/边界/取消能力和 Experimental/Formal 状态；
- 注册时拒绝空 ID、重复 ID、领域不一致和缺失必需元数据；
- 默认选择只接受 Formal 且通过对应特征发布门禁的算法；
- Job 执行指纹保存算法 ID + 行为版本，UI 名称不参与身份；
- 增加注册表枚举、重复/错误元数据、不可用选择和版本变化回归。

## 验收与维护

```powershell
ctest --test-dir build -C Release --output-on-failure --parallel 4 `
  -R "^(hole_.*|pocket_.*|profile_.*|app_workflow_controllers|module_boundaries)$"
```

新增或升级算法时先更新所属特征计划，再更新本文件的注册/准入信息和确定性测试。
