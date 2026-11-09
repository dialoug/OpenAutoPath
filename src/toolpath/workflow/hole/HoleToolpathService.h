// 孔刀路应用服务：独立负责参数归一化、孔算法调用和结构验证。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)的孔服务子目录。
// 职责:对孔生成做端到端编排——过滤未就绪输入、归一化参数、从注册表
// 解析算法、捕获算法异常/取消、执行孔专用验证并审计覆盖率与生产安全
// 证据。服务按值持有注册表与算法 id,可复制、无共享状态,线程安全。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/hole/HoleCoverage.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 孔生产安全证据:汇总实体扫掠碰撞检查的关键计数与六面装夹假设,
// 用于向用户展示「本次生成是否经过了生产级安全把关」。
struct HoleProductionSafetyEvidence {
    bool solidSweepMode = false;
    // 装配几何完整性通过诊断报告为警告；此证据只要求已知组件完成实体扫掠。
    // 当前生产范围采用六面装夹假设：加工一个面时默认其余五面被夹持，
    // 夹具、压板和机床结构不进入碰撞场景，也不作为导出门禁。
    bool sixFaceClampingAssumption = true;
    int workpieceBodyCount = 0;
    int fixtureBodyCount = 0;
    int machineBodyCount = 0;
    int solidSweepCheckCount = 0;
    int detectedCollisionCount = 0;

    bool complete() const noexcept {
        return solidSweepMode && sixFaceClampingAssumption &&
               workpieceBodyCount > 0 && solidSweepCheckCount > 0 &&
               detectedCollisionCount == 0;
    }
};

// 从工艺参数、碰撞场景与生成结果汇总生产安全证据。
// 参数:params 生效后的工艺参数;context 可选快速移动规划上下文
//       (其碰撞场景用于统计工件/夹具/机床实体数);machining 生成结果。
// 返回:evidence,其 complete() 反映是否满足生产导出门禁。
HoleProductionSafetyEvidence buildHoleProductionSafetyEvidence(
    const ToolpathParams& params,
    const HoleRapidPlanningContext* context,
    const HoleMachiningResult& machining);

// 孔刀路服务的一次完整输出:所用算法 id、生成结果、验证报告、生效参数、
// 诊断、覆盖率与生产安全证据。cancelled 为真时 operations 已清空。
struct HoleToolpathBatch {
    std::string algorithmId;
    HoleMachiningResult machining;
    ToolpathValidationReport validation;
    ToolpathParams effectiveParams;
    std::vector<ToolpathDiagnostic> diagnostics;
    HoleCoverageReport coverage;
    HoleProductionSafetyEvidence productionSafety;
    bool cancelled = false;

    // 把批内加工操作转换为中立的 ToolpathProgram(供后处理/导出)。
    ToolpathProgram toProgram() const;
};

// 孔刀路应用服务:对调用方隐藏算法解析与验证细节。
class HoleToolpathService {
public:
    // 默认构造:使用默认注册表与内置孔算法。
    HoleToolpathService();
    // 显式构造:注入注册表与算法 id(支持可替换实现)。
    HoleToolpathService(
        std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
        std::string algorithmId = kBuiltinHoleAlgorithmId);

    // 生成一批孔的刀路。
    // 参数:inputs 全部孔输入(未就绪的会被排除并记诊断);params 工艺
    //       参数;rapidContext 可选孔间快速移动规划上下文;
    //       strategyOverrides 可选逐孔策略覆盖;control 可选取消/进度。
    // 返回:完整批结果;取消/失败时通过 cancelled 与诊断表达,不抛异常。
    HoleToolpathBatch generate(
        const std::vector<HoleMachiningInput>& inputs,
        const ToolpathParams& params,
        const HoleRapidPlanningContext* rapidContext = nullptr,
        const std::vector<HoleMachiningStrategy>* strategyOverrides = nullptr,
        const ToolpathGenerationControl* control = nullptr) const;

private:
    // 仅对已就绪输入执行生成(内部步骤,由 generate 先过滤后调用)。
    HoleToolpathBatch generateReadyInputs(
        const std::vector<HoleMachiningInput>& holes,
        const ToolpathParams& params,
        const HoleRapidPlanningContext* rapidContext = nullptr,
        const std::vector<HoleMachiningStrategy>* strategyOverrides = nullptr,
        const ToolpathGenerationControl* control = nullptr) const;

    std::shared_ptr<const ToolpathAlgorithmRegistry> m_algorithms;
    std::string m_algorithmId;
};

} // namespace oap::toolpath
