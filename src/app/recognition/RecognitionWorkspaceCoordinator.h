// 识别状态、任务与 OCCT 呈现的应用边界。
// 职责:管理三轴结构识别任务生命周期(发起/进度/取结果)、识别会话
//   (结果/孔加工输入/通结构面等派生状态)、手动编辑与呈现。
// 模块归属:src/app(recognition 聚焦工作区协调器),由 MainWindow 持有。
// 协作:ThreeAxisRecognitionController 执行后台识别任务;
//   RecognitionPresentationCoordinator 负责把识别结果着色到场景,
//   InspectionOverlayPresenter 负责叠加层上的检查图元;
//   recognizeBaseFaces 直接调用 analysis 层分析器。
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <QString>

#include "app/workflows/shared/RecognitionSession.h"
#include "app/analysis/ThreeAxisRecognitionController.h"

class Bnd_Box;
class TopoDS_Face;
class gp_Pnt;
class gp_Trsf;
class Quantity_Color;

namespace oap {
struct MoldCoordinateFrame;
class SceneController;
struct Model;
class StepIndexer;
}

namespace oap::app::presentation {
class InspectionOverlayPresenter;
class OverlayController;
class RecognitionPresentationCoordinator;
}

namespace oap::app {

// 三轴识别结果安装状态:Installed 成功安装、NoModel 无模型、
// Cancelled 被取消、Failed 失败(以上均不动会话)。
enum class RecognitionInstallStatus {
    Installed,
    NoModel,
    Cancelled,
    Failed,
};

// 安装结果:状态、消息与是否产生组合呈现(供 UI 决定是否切换标签页)。
struct RecognitionInstallResult {
    RecognitionInstallStatus status = RecognitionInstallStatus::Failed;
    QString message;
    bool hasCombinedPresentation = false;

    bool installed() const noexcept
    {
        return status == RecognitionInstallStatus::Installed;
    }
};

// 手动识别编辑的最终状态:Updated 已应用、EmptySelection 未选中面、
// RequiresSingleFloor 槽/腔需恰好一个底面、InvalidFloor 底面无效、
// AmbiguousFloor 底面被多个实例引用、NoChange 无实际变化。
enum class ManualRecognitionEditStatus {
    Updated,
    EmptySelection,
    RequiresSingleFloor,
    InvalidFloor,
    AmbiguousFloor,
    NoChange,
};

// 手动编辑期间的警告类型:底面种子扩展失败 / 底面冲突。
enum class ManualRecognitionWarningKind {
    FloorExpansionFailed,
    ConflictingFloor,
};

// 一条手动编辑警告:类型、种子面 id、关联面 id 与细节文本。
struct ManualRecognitionWarning {
    ManualRecognitionWarningKind kind =
        ManualRecognitionWarningKind::FloorExpansionFailed;
    std::string seedFaceId;
    std::string relatedFaceId;
    std::string detail;
};

// 手动编辑结果:状态、受影响实例/面数、是否为清除型编辑(槽/腔)、
// 底面 id 与警告列表。
struct ManualRecognitionEditResult {
    ManualRecognitionEditStatus status =
        ManualRecognitionEditStatus::NoChange;
    int affectedInstanceCount = 0;
    int affectedFaceCount = 0;
    bool clearingKind = false;
    std::string floorFaceId;
    std::vector<ManualRecognitionWarning> warnings;

    bool updated() const noexcept
    {
        return status == ManualRecognitionEditStatus::Updated;
    }
};

// 基准面识别结果:是否已安装与基准面数量。
struct BaseFaceRecognitionResult {
    bool installed = false;
    int baseFaceCount = 0;
};

// 识别工作区协调器:聚合识别会话、后台任务控制器与两个呈现器。
// 状态所有权:RecognitionSession 保存识别结果及派生状态;
//   ThreeAxisRecognitionController 管理后台任务(线程边界);
//   呈现器负责场景着色与叠加层检查图元。
class RecognitionWorkspaceCoordinator final {
public:
    RecognitionWorkspaceCoordinator(
        presentation::OverlayController& overlays,
        ThreeAxisRecognitionController::Runner runner = {});
    ~RecognitionWorkspaceCoordinator();

    RecognitionWorkspaceCoordinator(
        const RecognitionWorkspaceCoordinator&) = delete;
    RecognitionWorkspaceCoordinator& operator=(
        const RecognitionWorkspaceCoordinator&) = delete;

    // 识别结果/派生状态只读查询:是否有结果、结果本身、孔加工输入、
    // 孔几何识别档案、基准面/通结构面集合、线追踪特征子集、
    // 槽源与槽区域缓存。
    bool hasResult() const noexcept { return m_session.hasResult(); }
    const RecognitionResult& result() const noexcept
    {
        return m_session.result();
    }
    bool hasHoleToolpathInputs() const noexcept
    {
        return m_session.hasHoleToolpathInputs();
    }
    const std::vector<toolpath::HoleMachiningInput>&
    holeToolpathInputs() const noexcept
    {
        return m_session.holeToolpathInputs();
    }
    const HoleGeometryRecognitionResult& holeGeometryProfile() const noexcept
    {
        return m_session.holeGeometryProfile();
    }
    const std::unordered_set<std::string>& baseFaces() const noexcept
    {
        return m_session.baseFaces();
    }
    const std::unordered_set<std::string>&
    throughStructureFaces() const noexcept
    {
        return m_session.throughStructureFaces();
    }
    const std::vector<ThreeAxisFeatureSubset>&
    wireTraceFeatureSubsets() const noexcept
    {
        return m_session.wireTraceFeatureSubsets();
    }
    const std::vector<toolpath::RootInnerWirePocketSource>&
    pocketSources() const noexcept
    {
        return m_session.pocketSources();
    }
    const std::vector<toolpath::RootOuterWireProfileSource>&
    profileSources() const noexcept
    {
        return m_session.profileSources();
    }
    const std::vector<toolpath::ProfileRegion>& ensureProfileRegionCache(
        const StepIndexer& stepIndexer);
    bool hasPocketRegionCache() const noexcept
    {
        return m_session.hasPocketRegionCache();
    }

    // 安装孔几何识别结果:把识别几何映射为孔加工输入列表并写回会话,
    // 返回写回的输入列表引用。
    const std::vector<toolpath::HoleMachiningInput>&
    installHoleGeometryRecognition(
        HoleGeometryRecognitionResult recognition,
        const Model& model,
        const MoldCoordinateFrame& moldFrame);
    // 确保槽区域缓存已构建(懒构建),返回缓存结果引用。
    const toolpath::PocketRegionWorkflowResult& ensurePocketRegionCache(
        const StepIndexer& stepIndexer);
    // 加载工程后同步工程中的已识别孔到本会话(与当前孔输入取并集)。
    void synchronizeLoadedProjectHoles(
        const std::vector<toolpath::HoleMachiningInput>& recognizedHoles);

    // 后台识别任务控制:start 返回请求 id,cancel 异步取消,
    // cancelAndWait 同步取消并等待;progress/ready/take 查询与取走结果。
    std::uint64_t start(ThreeAxisRecognitionRequest request);
    void cancel(std::uint64_t requestId = 0);
    void cancelAndWait();
    ThreeAxisRecognitionProgress progress(std::uint64_t requestId) const;
    bool ready(std::uint64_t requestId) const;
    std::optional<ThreeAxisRecognitionTaskResult> take(
        std::uint64_t requestId);

    // 模型切换:取消并等待后台任务,重置会话/呈现并清空叠加层。
    void resetForModel();
    // 安装三轴识别结果到会话并重建呈现:先做 NoModel/取消/失败门禁,
    // 在候选会话上装配派生状态后原子替换,再重置场景颜色并重建/应用
    // 识别呈现。beforeCommit 在替换前同步回调。
    RecognitionInstallResult installThreeAxisResult(
        const RecognitionResult& recognition,
        ThreeAxisWireTraceUseCaseResult useCaseResult,
        const Model& model,
        const MoldCoordinateFrame& moldFrame,
        SceneController& scene,
        qint64 elapsedMillis,
        const std::function<void()>& beforeCommit = {});

    // 手动识别编辑:把选中面设为/清除为指定特征(槽/腔需恰一个底面),
    // 变更应用到会话并失效相关派生状态;返回编辑结果与警告。
    ManualRecognitionEditResult applyManualFeatureEdit(
        FeatureKind kind,
        const std::vector<std::string>& selectedFaceIds,
        const Model& model);
    // 清除选中面所属的识别实例。
    ManualRecognitionEditResult clearFeatureSelection(
        const std::vector<std::string>& selectedFaceIds);
    // 识别基准面(单元外侧面),结果写入会话。
    BaseFaceRecognitionResult recognizeBaseFaces(
        const Model& model,
        const StepIndexer& stepIndexer);

    // 把识别颜色应用到场景(孔/通结构等特征面着色)。
    void applyRecognitionColors(SceneController& scene) const;
    // 是否已有识别呈现内容。
    bool hasRecognitionPresentation() const noexcept;

    // 叠加层检查图元管理:清空、追加检查面、见证线与包围盒。
    void clearInspection();
    void appendInspectionFace(
        const TopoDS_Face& face,
        const Quantity_Color& color,
        const gp_Trsf& modelTransform);
    bool appendWitnessLine(
        const gp_Pnt& first,
        const gp_Pnt& second,
        const Quantity_Color& color,
        double width,
        double coincidenceTolerance,
        const gp_Trsf& modelTransform);
    bool appendInspectionBoundingBox(
        const Bnd_Box& bounds,
        const Quantity_Color& color,
        double width,
        const gp_Trsf& modelTransform);

private:
    // 手动编辑后失效的派生状态:孔加工输入、孔几何档案与槽区域缓存。
    void invalidateManualEditDerivatives();

    RecognitionSession m_session;      // 识别结果及派生状态(长期状态)。
    ThreeAxisRecognitionController m_controller;   // 后台任务控制器。
    std::unique_ptr<presentation::RecognitionPresentationCoordinator>
        m_presentation;               // 识别结果场景着色。
    std::unique_ptr<presentation::InspectionOverlayPresenter>
        m_inspectionPresenter;        // 叠加层检查图元。
};

} // namespace oap::app
