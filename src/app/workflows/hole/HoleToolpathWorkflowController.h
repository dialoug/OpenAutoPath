// 生成孔程序前的与 UI 无关的准备与原子提交。
// 属 oap_app_workflow_cam 孔工作流模块（src/app/workflows/hole），
// 由 HoleToolpathGenerationUseCase 调用；prepareGeometry 负责几何准备，
// commit 负责把生成结果原子写入 MachiningJob 。
#pragma once

#include <vector>

#include <QString>

#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "toolpath/hole/HoleMoldEnvelopeEntryResolver.h"
#include "toolpath/hole/HoleMachining.h"
#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"

namespace oap::app {

// 孔刀路几何准备结果：由 prepareGeometry 一次性构造，
// 包含外包络入口恢复结果、识别孔源与快速避障上下文。
struct HoleToolpathGeometryPreparation {
    toolpath::HoleMoldEnvelopeEntryResult envelope;
    std::vector<RecognizedHoleSource> sources;
    toolpath::HoleRapidPlanningContext rapidContext;
    int obstacleShapeCount = 0;
};

// 原子提交请求：所有指针字段指向调用方保持存活的输入；
// currentJob 为基线 Job，提交成功或失败均不修改这些输入。
struct HoleToolpathCommitRequest {
    const manufacturing::MachiningJob* currentJob = nullptr;
    MachiningProjectDefinitionContext definition;
    // Empty preserves the common Job part identity initialized by ensureDefinition.
    QString recognitionProfileVersion;
    const std::vector<toolpath::HoleMachiningInput>* inputs = nullptr;
    const tooling::HoleToolSelectionPlan* automaticSelection = nullptr;
    const toolpath::HoleToolpathBatch* generated = nullptr;
    const toolpath::MoldSetupWorkFrames* setupFrames = nullptr;
};

// 提交结果：ok 为 true 时 job 已替换为新状态；
// 否则 error 说明失败原因，调用方数据保持不变。
struct HoleToolpathCommitResult {
    bool ok = false;
    QString error;
    manufacturing::MachiningJob job;
};

// 孔程序工作流控制器（无状态静态门面）：把「几何准备」与「原子提交」
// 拆成两步，供自动选刀生成路径使用。
class HoleToolpathWorkflowController {
public:
    // 从识别结果准备刀路几何：解析模具外包络入口、重建孔源阶段键，
    // 并基于模型实体收集障碍体以构建快速避障上下文。
    static HoleToolpathGeometryPreparation prepareGeometry(
        const std::vector<toolpath::HoleMachiningInput>& recognized,
        const Model& model,
        const StepIndexer& indexer,
        const Bbox& moldBounds,
        const MoldCoordinateFrame& moldFrame);
    // 原子提交：先在 Job 副本上完成定义同步与 HoleJob 捕获，
    // 全部成功后整体替换 Job。
    static HoleToolpathCommitResult commit(
        const HoleToolpathCommitRequest& request);
};

} // namespace oap::app
