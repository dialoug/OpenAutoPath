// FreeCAD Job 风格的加工项目根模型。可持久化定义只保存重建刀路所需的
// 稳定值；OCCT 刀路和仿真结果作为运行期缓存，不写入项目 JSON。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gp_Dir.hxx>

#include "analysis/FeatureTypes.h"
#include "manufacturing/ipw/IpwCheckpointStore.h"
#include "manufacturing/ipw/IpwSession.h"
#include "toolpath/core/MachiningSetup.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathToolContext.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"
#include "toolpath/finishing/SurfaceFinishingBatch.h"

namespace oap::toolpath { class StockTargetCache; }
namespace oap::manufacturing {

// 可读取的最低 .oapjob schema 版本；低于此版本的旧文件直接拒绝加载。
inline constexpr int kMinimumSupportedMachiningJobSchemaVersion = 1;
// V10 introduces OpenCAMLib finishing with a new schema, distinct from removed legacy surfaces.
// V12 preserves layered engraving inputs; older executables must not silently
// regenerate a stock-removal operation as finish-only.
inline constexpr int kMachiningJobSchemaVersion = 12;

// 与 OCCT gp_Pnt/gp_Dir 互转的稳定 XYZ 表示，用于 JSON 持久化与指纹计算。
struct JobVector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

JobVector3 toJobVector(const gp_Pnt& point) noexcept;
JobVector3 toJobVector(const gp_Dir& direction) noexcept;
gp_Pnt toPoint(const JobVector3& value) noexcept;

// 毛坯来源：按零件包围盒各向扩展、直接取模型包围盒，或引用外部 STEP 毛坯。
enum class StockSourceKind {
    PerUnitBoxes,
    ModelBox,
    StepModel,
};

// 工序生命周期状态：Planned 待生成，Dirty 定义已变，Generated 已生成且有效，
// Warning 生成含警告，Error 生成失败，Stale 输入失效，NeedsReview 需人工复核。
enum class JobOperationState {
    Planned,
    Dirty,
    Generated,
    Warning,
    Error,
    Stale,
    NeedsReview,
};

const char* stockSourceKindId(StockSourceKind kind) noexcept;
const char* jobOperationStateId(JobOperationState state) noexcept;

// 工件零件来源定义：记录源文件路径、内容指纹与识别配置版本，
// 加载后用于判定是否需要重新识别/重绑定。
struct PartDefinition {
    std::string sourcePath;
    std::string sourceFingerprint;
    std::string units = "mm";
    std::string recognitionProfileVersion;
};

// 毛坯定义：来源类型、可选 STEP 路径及各方向扩展量（单位与零件一致）。
struct StockDefinition {
    StockSourceKind sourceKind = StockSourceKind::PerUnitBoxes;
    std::string sourcePath;
    double marginX = 0.0;
    double marginY = 0.0;
    double marginZ = 0.0;
    // Content identity of an external planning stock STEP, bound when loaded.
    std::string sourceFingerprint;
};

// 后置轮廓 NC 输出引用：输出文件标识、版本与 GCode 导出选项。
struct PostProfileReference {
    std::string id = "reference.nc";
    std::string version = "1";
    toolpath::GCodeProgramOptions options;
};

// 碰撞模型引用：参与机床/夹具仿真的外部 STEP 模型及其刚体定位。
struct CollisionModelReference {
    std::string sourcePath;
    std::string sourceFingerprint;
    toolpath::CollisionBodyRole role =
        toolpath::CollisionBodyRole::Fixture;
    // STEP 模型坐标到当前工件/机床场景坐标的刚体定位（角度为度）。
    JobVector3 translation;
    JobVector3 rotationDeg;
};

// 碰撞场景定义：参与机床/夹具仿真的外部模型引用集合及人工确认标记。
struct CollisionSceneDefinition {
    std::vector<CollisionModelReference> models;
    bool confirmed = false;
};

// 装夹确认记录：确认标记、确认人与绑定输入指纹（输入变化后失效）。
struct SetupConfirmation {
    bool confirmed = false;
    std::string confirmedBy;
    std::string confirmedAt;
    // 确认时的模型、原点、轴向和 WCS 指纹；任一输入变化后不得沿用确认。
    std::string confirmedInputFingerprint;
};

// 六面装夹定义：记录工作坐标系、原点/轴向及人工确认信息；
// confirmation 与零件/装夹输入指纹绑定，输入变化后失效。
struct SetupDefinition {
    std::string id;
    int setupIndex = 0;
    MoldSetupFace face = MoldSetupFace::Unresolved;
    toolpath::WorkCoordinateSystem workOffset =
        toolpath::WorkCoordinateSystem::None;
    JobVector3 origin;
    JobVector3 xAxis{1.0, 0.0, 0.0};
    JobVector3 yAxis{0.0, 1.0, 0.0};
    JobVector3 zAxis{0.0, 0.0, 1.0};
    bool valid = false;
    bool requiresPhysicalReclamp = false;
    std::string axisMapping;
    std::string operatorInstruction;
    SetupConfirmation confirmation;
};

// Job 层刀具控制器是创建工序时的不可变快照。它引用刀库来源，同时保留
// 实际 T 号、切削参数和刀具装配几何，确保刀库后续修改不会静默改变旧 Job。
struct JobToolController {
    std::string id;
    std::string libraryId;
    toolpath::ToolpathToolContext tool;
};

// 持久化完整孔几何，同时保留稳定面 ID 和几何签名。重新识别后应先按
// featureId 匹配，失败时再使用 geometryFingerprint 做容差重绑定。
struct HoleTargetReference {
    std::string featureId;
    std::string geometryFingerprint;
    int sourceHoleIndex = -1;
    std::vector<int> coveredSourceHoleIndices;
    int physicalHoleIndex = -1;
    toolpath::HoleMachiningInput hole;
    toolpath::HoleMachiningStrategy strategy =
        toolpath::HoleMachiningStrategy::Auto;
};

// 一道 Job 工序对应同装夹、同刀具、同阶段的一组孔。底层生成器仍可把它
// 展开成多个 ToolpathOperation，但排序、启用和参数来源由此定义控制。
struct HoleOperationDefinition {
    std::string id;
    std::string setupId;
    std::string toolControllerId;
    toolpath::HoleMachiningPhase phase =
        toolpath::HoleMachiningPhase::Standard;
    std::vector<HoleTargetReference> targets;
    bool enabled = true;
    int sequence = 0;
    JobOperationState state = JobOperationState::Planned;
    std::string inputFingerprint;
    // 独立局部批次在保存和重建后仍保持来源，不被全量批次吸收。
};

// 槽腔工序只保存可重新识别的稳定拓扑引用。清根环、下刀段、连接移动和
// OCCT/Dexel 结果均为派生数据，不进入 Job JSON。
enum class PocketRegionSourceKind {
    FeatureFaces,
    RootInnerWire,
};

// 槽腔目标稳定拓扑引用：可重新识别的来源面/根环集合与几何指纹。
struct PocketTargetReference {
    PocketRegionSourceKind sourceKind =
        PocketRegionSourceKind::FeatureFaces;
    FeatureKind kind = FeatureKind::Pocket;
    std::vector<std::string> sourceFaceIds;
    std::string floorFaceId;
    // RootInnerWire 来源必须保留根面/根内环，普通 FeatureFaces 留空。
    std::string rootFaceId;
    std::string rootWireId;
    bool allowOpenSectionContours = false;
    int openBoundaryFaceCount = 0;
    std::string geometryFingerprint;
};

// 槽腔工序定义：装夹/刀具/算法与目标引用，供 Job 重建与展开。
struct PocketOperationDefinition {
    std::string id;
    std::string setupId;
    std::string toolControllerId;
    std::string algorithmId;
    double restMachiningPreviousToolDiameter = 0.0;
    PocketTargetReference target;
    bool enabled = true;
    // sequence 与孔工序共享同一个全局序号空间。
    int sequence = 0;
    JobOperationState state = JobOperationState::Planned;
    std::string inputFingerprint;
};

// 外轮廓目标只保存可从 STEP 稳定重建的根面/真实 outer wire 与侧壁面集合；
// 刀路移动仍属于运行期派生数据，不写入 Job。
struct ProfileTargetReference {
    std::string axisLabel;
    int normalDirection = 1;
    std::vector<std::string> sourceFaceIds;
    std::vector<std::string> machiningFaceIds;
    std::vector<std::string> terminalFaceIds;
    std::vector<std::string> mainOuterFaceIds;
    std::string rootFaceId;
    std::string rootWireId;
    std::string geometryFingerprint;
};

// 外轮廓工序定义：装夹/刀具/算法、目标引用与多刀余量加工参数。
struct ProfileOperationDefinition {
    std::string id;
    std::string setupId;
    std::string toolControllerId;
    std::string algorithmId;
    ProfileTargetReference target;
    // 0 表示全量外部清料；正值表示本工序只加工该前序大刀留下的余料。
    double restMachiningPreviousToolDiameter = 0.0;
    // 多刀链中只有末刀保留最终轮廓精修，避免每个阶段重复贴壁。
    bool finishPass = true;
    bool enabled = true;
    int sequence = 0;
    JobOperationState state = JobOperationState::Planned;
    std::string inputFingerprint;
};

// Job 可序列化定义（聚合根的持久化部分）：保存重建刀路所需全部稳定值，
// 不含 OCCT 刀路移动与仿真结果；序列化见 MachiningJobStorage。
struct SurfaceFinishingTargetReference {
    std::vector<std::string> unitIds;
    std::string geometryFingerprint;
};

struct SurfaceFinishingOperationDefinition {
    std::string id;
    std::string setupId;
    std::string toolControllerId;
    std::string algorithmId = toolpath::kOpenCamLibSurfaceFinishingAlgorithmId;
    SurfaceFinishingTargetReference target;
    toolpath::SurfaceFinishingParams parameters;
    bool enabled = true;
    int sequence = 0;
    JobOperationState state = JobOperationState::Planned;
    std::string inputFingerprint;
};

struct MachiningJobDefinition {
    int schemaVersion = kMachiningJobSchemaVersion;
    std::string id;
    std::string name;
    std::string createdAt;
    std::string modifiedAt;
    PartDefinition part;
    StockDefinition stock;
    PostProfileReference post;
    CollisionSceneDefinition collisionScene;
    std::string activeToolLibraryId;
    std::string algorithmId = "hole.auto_tool_library";

    toolpath::ToolpathParams holeParameters;
    toolpath::ToolpathParams pocketParameters;
    toolpath::ToolpathParams profileParameters;
    // 保存识别阶段的完整孔段集合，而不只保存已成功规划的工序目标。这样 Job
    // 重新打开后仍能发现当时因无刀或装夹未解析而漏掉的孔。
    std::vector<toolpath::HoleMachiningInput> recognizedHoles;
    int recognizedPhysicalHoleCount = 0;
    std::vector<SetupDefinition> setups;
    std::vector<JobToolController> toolControllers;
    std::vector<HoleOperationDefinition> holeOperations;
    std::vector<PocketOperationDefinition> pocketOperations;
    std::vector<ProfileOperationDefinition> profileOperations;
    std::vector<SurfaceFinishingOperationDefinition> finishingOperations;
};

// Job 运行期缓存：生成的刀路批次、IPW 会话与检查点均不写入项目 JSON，
// 由 invalidateRuntime() 整体重置，或由孔/槽腔 Job 服务重建。
struct MachiningJobRuntime {
    std::shared_ptr<toolpath::StockTargetCache> stockTargetCache;
    toolpath::HoleToolpathBatch holeResult;
    toolpath::PocketToolpathBatch pocketResult;
    toolpath::ProfileToolpathBatch profileResult;
    toolpath::SurfaceFinishingBatch finishingResult;
    ipw::IpwSession ipwSession;
    ipw::IpwCheckpointStore ipwCheckpoints;
    std::string holeExecutionFingerprint;
    std::string pocketExecutionFingerprint;
    std::string profileExecutionFingerprint;
    std::string finishingExecutionFingerprint;
    bool hasHoleResult = false;
    bool hasPocketResult = false;
    bool hasProfileResult = false;
    bool hasFinishingResult = false;
};

// 加工项目聚合根：同时持有可序列化定义与运行期缓存，是 Job 存储、应用服务
// 和孔/槽腔服务操作的唯一入口。默认构造后为空 Job，需由存储加载或捕获填充。
// Stable mapping used by IPW execution and presentation replay.
bool runtimeOperationIdMatches(const std::string& definitionId,
                               const std::string& runtimeId);

class MachiningJob {
public:
    MachiningJobDefinition& definition() noexcept { return m_definition; }
    const MachiningJobDefinition& definition() const noexcept {
        return m_definition;
    }

    MachiningJobRuntime& runtime() noexcept { return m_runtime; }
    const MachiningJobRuntime& runtime() const noexcept { return m_runtime; }

    bool hasHoleResult() const noexcept;
    const toolpath::HoleToolpathBatch* holeResult() const noexcept;
    toolpath::HoleToolpathBatch* holeResult() noexcept;
    void setHoleResult(toolpath::HoleToolpathBatch result,
                       std::string executionFingerprint);
    void clearHoleResult();
    bool hasPocketResult() const noexcept;
    // 消费入口：拒绝缺失、取消、校验失败或输入指纹过期的结果。
    // 返回借用指针；调用期间不得修改 Job。原始 pocketResult() 仍供诊断使用。
    const toolpath::PocketToolpathBatch* currentPocketResult(
        std::string* error = nullptr) const;
    const toolpath::PocketToolpathBatch* pocketResult() const noexcept;
    toolpath::PocketToolpathBatch* pocketResult() noexcept;
    void setPocketResult(toolpath::PocketToolpathBatch result,
                         std::string executionFingerprint);
    void clearPocketResult();
    bool hasProfileResult() const noexcept;
    const toolpath::ProfileToolpathBatch* profileResult() const noexcept;
    toolpath::ProfileToolpathBatch* profileResult() noexcept;
    void setProfileResult(toolpath::ProfileToolpathBatch result,
                          std::string executionFingerprint);
    void clearProfileResult();
    bool hasFinishingResult() const noexcept;
    const toolpath::SurfaceFinishingBatch* finishingResult() const noexcept;
    toolpath::SurfaceFinishingBatch* finishingResult() noexcept;
    const toolpath::SurfaceFinishingBatch* currentFinishingResult(std::string* error = nullptr) const;
    void setFinishingResult(toolpath::SurfaceFinishingBatch result, std::string executionFingerprint);
    void clearFinishingResult();
    // 当前 Job 的孔/槽腔持久化工序按共享的全局 sequence 映射到 IPW。
    // 返回包含更新结果；includedOperationIds 非空时只选择其中存在的工序，
    // 缺省为空表示全部启用工序。
    ipw::IpwPlanUpdate configureIpwPlan(
        std::string initialStockFingerprint,
        std::string engineFingerprint,
        const std::vector<std::string>& includedOperationIds = {});
    ipw::IpwSession& ipwSession() noexcept
    {
        return m_runtime.ipwSession;
    }
    const ipw::IpwSession& ipwSession() const noexcept
    {
        return m_runtime.ipwSession;
    }
    // 丢弃全部运行期缓存（刀路结果与 IPW），保留可序列化定义。
    void invalidateRuntime();
    // 清空整个 Job（定义与运行期），回到空 Job 初始状态。
    void clear();

    const SetupDefinition* setupById(const std::string& id) const noexcept;
    const JobToolController* toolControllerById(
        const std::string& id) const noexcept;

    // 只检查可序列化结构和引用完整性，不代替刀路/机床安全验证。
    std::vector<std::string> validationErrors() const;

private:
    MachiningJobDefinition m_definition;
    MachiningJobRuntime m_runtime;
};

} // namespace oap::manufacturing
