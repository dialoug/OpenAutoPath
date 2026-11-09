#include "app/workflows/engraving/EngravingToolpathGenerationUseCase.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <tuple>

#include <BRep_Builder.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>

#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "tooling/adapter/AssignedToolContext.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/core/ToolpathParallel.h"

namespace oap::app {
namespace {
class BusyGuard {
public:
    explicit BusyGuard(const WorkflowBusyStateHandler& handler) : m_handler(handler)
    {
        if (m_handler) m_handler(true);
    }
    ~BusyGuard() { if (m_handler) m_handler(false); }
private:
    WorkflowBusyStateHandler m_handler;
};

int cutterPriority(tooling::ToolType type)
{
    switch (type) {
    case tooling::ToolType::BallEndMill: return 0;
    case tooling::ToolType::BullNose: return 1;
    case tooling::ToolType::EndMill: return 2;
    default: return 3;
    }
}

bool validToolGeometry(const toolpath::ToolpathToolContext& tool, tooling::ToolType type)
{
    tooling::ToolGeometry geometry;
    geometry.diameter = tool.diameter;
    geometry.cuttingEdgeLength = tool.cuttingEdgeLength;
    geometry.overallLength = tool.overallLength;
    geometry.cornerRadius = tool.cornerRadius;
    geometry.tipAngle = tool.tipAngle;
    geometry.tipCompensation = tool.tipCompensation;
    geometry.shankDiameter = tool.shankDiameter;
    geometry.holderDiameter = tool.holderDiameter;
    geometry.holderLength = tool.holderLength;
    geometry.spindleEnvelopeDiameter = tool.spindleEnvelopeDiameter;
    geometry.spindleEnvelopeLength = tool.spindleEnvelopeLength;
    return tooling::toolGeometryValidationErrors(type, geometry).isEmpty();
}

bool bindTool(const EngravingToolpathGenerationRequest& request,
    toolpath::ToolpathToolContext& tool, toolpath::SurfaceFinishingParams& params,
    std::string& libraryId, QString& error)
{
    params = request.settings.parameters;
    if (request.settings.toolMode == EngravingToolMode::SingleTool) {
        tool = request.settings.manualTool;
        if (tool.toolId.empty()) tool.toolId = "engraving-single-tool";
        if (tool.toolName.empty()) tool.toolName = "Engraving single tool";
    } else if (request.settings.toolMode == EngravingToolMode::AutomaticLibrary) {
        if (!std::isfinite(params.diameter) || params.diameter <= 0 ||
            !std::isfinite(params.stepOver) || params.stepOver <= 0) {
            error = QStringLiteral("雕刻自动选刀需要有效的刀径上限及行距。");
            return false;
        }
        if (!request.library) {
            error = QStringLiteral("雕刻刀库不可用，请在刀路参数中选择刀库或配置单刀。");
            return false;
        }
        std::vector<toolpath::ToolpathToolContext> candidates;
        for (const auto& assigned : request.library->assignedTools()) {
            if (cutterPriority(assigned.tool.type) == 3 ||
                !tooling::toolGeometryValidationErrors(assigned.tool.type, assigned.tool.geometry).isEmpty())
                continue;
            auto candidate = tooling::makeToolpathToolContext(assigned);
            if (!candidate.valid || candidate.cuttingEdgeLength <= 0 ||
                candidate.diameter > params.diameter || candidate.diameter < params.stepOver)
                continue;
            candidates.push_back(std::move(candidate));
        }
        if (candidates.empty()) {
            error = QStringLiteral("雕刻刀库中没有满足刀径上限、行距及刃长要求的已分配球头刀、圆鼻刀或平底刀。");
            return false;
        }
        std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            const auto key = [](const auto& value) {
                return std::tuple{cutterPriority(tooling::toolTypeFromId(QString::fromStdString(value.toolType))),
                    value.diameter, value.toolNumber, value.toolId};
            };
            return key(a) < key(b);
        });
        tool = std::move(candidates.front());
        libraryId = request.library->id().toStdString();
        // Zero feed in a library is an explicit request to use the configured
        // process default, frozen into the resulting controller snapshot.
        if (tool.horizontalFeed == 0) tool.horizontalFeed = params.feed;
        if (tool.verticalFeed == 0) tool.verticalFeed = params.plungeFeed;
    } else {
        error = QStringLiteral("雕刻刀具来源无效。");
        return false;
    }
    const auto type = tooling::toolTypeFromId(QString::fromStdString(tool.toolType));
    if (!tool.valid || tool.toolNumber <= 0 || cutterPriority(type) == 3 || !validToolGeometry(tool, type) ||
        !std::isfinite(tool.diameter) || tool.diameter <= 0 ||
        !std::isfinite(tool.cuttingEdgeLength) || tool.cuttingEdgeLength <= 0 ||
        !std::isfinite(tool.horizontalFeed) || tool.horizontalFeed <= 0 ||
        !std::isfinite(tool.verticalFeed) || tool.verticalFeed <= 0 ||
        !std::isfinite(tool.spindleSpeed) || tool.spindleSpeed < 0 ||
        !std::isfinite(tool.horizontalRapid) || tool.horizontalRapid < 0 ||
        !std::isfinite(tool.verticalRapid) || tool.verticalRapid < 0 ||
        !std::isfinite(tool.peckDepth) || tool.peckDepth < 0) {
        error = QStringLiteral("雕刻单刀或刀库快照无效：需要有效刀型、正刀号、刀径、刃长和切削进给。");
        return false;
    }
    params.cutterKind = type == tooling::ToolType::BallEndMill ? toolpath::FinishingCutterKind::Ball :
        type == tooling::ToolType::BullNose ? toolpath::FinishingCutterKind::Bull : toolpath::FinishingCutterKind::Flat;
    if (params.cutterKind == toolpath::FinishingCutterKind::Ball) tool.cornerRadius = tool.diameter / 2;
    if (params.cutterKind == toolpath::FinishingCutterKind::Flat) tool.cornerRadius = 0;
    params.diameter = tool.diameter;
    params.cuttingLength = tool.cuttingEdgeLength;
    params.cornerRadius = tool.cornerRadius;
    params.feed = tool.horizontalFeed;
    params.plungeFeed = tool.verticalFeed;
    return true;
}
} // namespace

EngravingToolpathGenerationResult EngravingToolpathGenerationUseCase::execute(
    const EngravingToolpathGenerationRequest& request)
{
    EngravingToolpathGenerationResult rejected;
    if (!request.setupFrames || request.targets.empty() || request.unitIds.empty() ||
        request.targets.size() != request.unitIds.size() ||
        std::any_of(request.targets.begin(), request.targets.end(), [](const auto& shape) { return shape.IsNull(); })) {
        rejected.error = QStringLiteral("雕刻需要完整模型实体及装夹坐标；无需先识别孔或槽腔。");
        return rejected;
    }
    std::vector<MoldSetupFace> faces;
    switch (request.settings.scope) {
    case EngravingSetupScope::PositiveZ:
        faces.push_back(MoldSetupFace::PositiveZ);
        break;
    case EngravingSetupScope::NegativeX:
        faces.push_back(MoldSetupFace::NegativeX);
        break;
    case EngravingSetupScope::PositiveX:
        faces.push_back(MoldSetupFace::PositiveX);
        break;
    case EngravingSetupScope::NegativeY:
        faces.push_back(MoldSetupFace::NegativeY);
        break;
    case EngravingSetupScope::PositiveY:
        faces.push_back(MoldSetupFace::PositiveY);
        break;
    case EngravingSetupScope::NegativeZ:
        faces.push_back(MoldSetupFace::NegativeZ);
        break;
    case EngravingSetupScope::AllSixFaces:
        for (int index = 0; index < 6; ++index) faces.push_back(static_cast<MoldSetupFace>(index));
        break;
    case EngravingSetupScope::PreserveProjectFaces:
        faces = request.settings.projectFaces;
        std::sort(faces.begin(), faces.end());
        faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
        if (faces.empty()) {
            rejected.error = QStringLiteral("项目既有雕刻装夹为空。");
            return rejected;
        }
        break;
    default:
        rejected.error = QStringLiteral("雕刻装夹范围无效。");
        return rejected;
    }
    for (const auto face : faces) {
        const auto* frame = request.setupFrames->forFace(face);
        if (!frame || !frame->valid) {
            rejected.error = QStringLiteral("雕刻装夹坐标无效，请先建立工件坐标。");
            return rejected;
        }
    }
    toolpath::SurfaceFinishingParams params;
    if (!bindTool(request, rejected.tool, params, rejected.libraryId, rejected.error))
        return rejected;
    const auto tool = rejected.tool;
    const auto libraryId = rejected.libraryId;
    const auto frames = *request.setupFrames;
    auto job = request.currentJob ? *request.currentJob : manufacturing::MachiningJob{};
    BusyGuard busy(request.interaction.setBusy);
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    auto completed = std::make_shared<std::atomic_size_t>(0);
    auto future = toolpath::submitHeavyJob(cancellation,
        [targets = request.targets, unitIds = request.unitIds, definition = request.definition,
         job = std::move(job), faces, frames, params, tool, libraryId, cancellation, completed]() mutable {
            EngravingToolpathGenerationResult result;
            result.tool = tool;
            result.libraryId = libraryId;
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
                if (!MachiningProjectController::ensureDefinition(job, definition, &result.error)) return result;
                BRep_Builder builder;
                TopoDS_Compound target;
                builder.MakeCompound(target);
                for (const auto& shape : targets) builder.Add(target, shape);
                toolpath::ToolpathGenerationControl control;
                toolpath::attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] { return cancellation->load(std::memory_order_relaxed); };
                std::string stockError;
                const auto stock = manufacturing::SurfaceFinishingMachiningJobService::buildPlanningStock(
                    job.definition().stock, targets, &stockError, &control);
                if (control.cancelled()) { result.cancelled = true; return result; }
                if (stock.IsNull()) {
                    result.error = QStringLiteral("雕刻规划毛坯无效：%1").arg(QString::fromStdString(stockError));
                    return result;
                }
                std::vector<toolpath::SurfaceFinishingResult> generated;
                generated.reserve(faces.size());
                const bool layered = params.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing;
                std::size_t remainingMoves = params.maxMoves;
                for (const auto face : faces) {
                    if (control.cancelled()) { result.cancelled = true; return result; }
                    auto faceParams = params;
                    if (layered) {
                        if (remainingMoves < 4) {
                            result.error = QStringLiteral("雕刻全部装夹的累计移动数量已达到预算；请缩小加工范围或调整采样参数。");
                            return result;
                        }
                        faceParams.maxMoves = remainingMoves;
                    }
                    auto surface = toolpath::SurfaceFinishing().generate(target,
                        frames.forFace(face)->setupWorkFrame, faceParams, &control, &stock);
                    if (surface.cancelled || control.cancelled()) { result.cancelled = true; return result; }
                    if (!surface.ok) {
                        result.error = QStringLiteral("雕刻 %1 生成失败：%2")
                            .arg(QString::fromLatin1(moldSetupFaceName(face)), QString::fromStdString(surface.error));
                        return result;
                    }
                    if (layered) {
                        if (surface.moves.size() > remainingMoves) {
                            result.error = QStringLiteral("雕刻全部装夹的累计移动数量超过预算。");
                            return result;
                        }
                        remainingMoves -= surface.moves.size();
                        surface.effectiveParams.maxMoves = params.maxMoves;
                    }
                    generated.push_back(std::move(surface));
                    completed->fetch_add(1, std::memory_order_relaxed);
                }
                std::vector<manufacturing::SurfaceFinishingJobCaptureRequest> captures;
                for (std::size_t index = 0; index < faces.size(); ++index) {
                    manufacturing::SurfaceFinishingJobCaptureRequest capture;
                    capture.result = &generated[index];
                    capture.tool = &tool;
                    capture.setupFrames = &frames;
                    capture.setupFace = faces[index];
                    capture.unitIds = unitIds;
                    capture.libraryId = libraryId;
                    captures.push_back(std::move(capture));
                }
                std::string error;
                if (!manufacturing::SurfaceFinishingMachiningJobService().replaceFamily(job, captures, &error)) {
                    result.error = QString::fromStdString(error);
                    return result;
                }
                if (control.cancelled()) { result.cancelled = true; return result; }
                if (const auto* batch = job.currentFinishingResult()) result.batch = *batch;
                result.program = result.batch.toProgram();
                result.summary = QStringLiteral("雕刻加工：%1 个装夹方向，T%2，刀径 %3 mm，%4 段移动。")
                    .arg(faces.size()).arg(tool.toolNumber).arg(tool.diameter, 0, 'f', 3).arg(result.program.moveCount());
                result.job = std::move(job);
                result.ok = true;
            } catch (const Standard_Failure& failure) {
                result.error = QString::fromUtf8(failure.GetMessageString());
            } catch (const std::exception& failure) {
                result.error = QString::fromUtf8(failure.what());
            } catch (...) {
                result.error = QStringLiteral("雕刻生成发生未知错误。");
            }
            return result;
        }, [](toolpath::HeavyJobRejection reason) {
            EngravingToolpathGenerationResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            result.error = QString::fromStdString(toolpath::heavyJobRejectionMessage(reason));
            return result;
        });
    toolpath::HeavyJobWaitGuard wait(future, cancellation);
    WorkflowProgressRequest progress;
    progress.title = QStringLiteral("雕刻加工");
    progress.label = QStringLiteral("正在从完整模型生成雕刻刀路…");
    progress.cancellingLabel = QStringLiteral("正在取消雕刻，等待当前几何计算结束…");
    progress.initialTotal = faces.size();
    progress.snapshot = [completed, count = faces.size()] {
        return WorkflowProgressSnapshot{completed->load(std::memory_order_relaxed), count, {}};
    };
    progress.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    progress.ready = [&future] { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; };
    progress.cancel = [cancellation] { toolpath::cancelHeavyJob(cancellation); };
    progress.cancellationRequested = [cancellation] { return cancellation->load(std::memory_order_relaxed); };
    waitForWorkflowTask(request.interaction.showProgress, progress);
    auto result = future.get();
    if (result.cancelled && result.error.isEmpty()) result.error = QStringLiteral("雕刻生成已取消，原加工项目保持不变。");
    return result;
}

} // namespace oap::app
