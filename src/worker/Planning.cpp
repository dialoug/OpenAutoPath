#include "Planning.h"
#include "analysis/ModelIndex.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "manufacturing/pocket/PocketMachiningJobService.h"
#include "manufacturing/profile/ProfileMachiningJobService.h"
#include "recognition/FeatureRecognizer.h"
#include "tooling/adapter/AssignedToolContext.h"
#include "tooling/adapter/pocket/PocketAutomaticToolpath.h"
#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include <stdexcept>

namespace oap::worker
{
using namespace manufacturing;

ThreeAxisWireTraceUseCaseResult traceRegions(const Model &model, const StepIndexer &index,
                                             const std::vector<RecognizedHoleGeometry> &holes)
{
    ModelIndex lookup;
    lookup.build(model);
    ThreeAxisWireTraceRequest request;
    request.model = &model;
    request.stepIndexer = &index;
    request.modelIndex = &lookup;
    for (const auto &h : holes)
        request.recognizedHoleFaceIds.insert(h.sourceFaceIds.begin(), h.sourceFaceIds.end());
    return ThreeAxisWireTraceUseCase().execute(request);
}

namespace
{
QString featureId(const char *kind, const std::string &face, const std::string &wire)
{
    return QString::fromLatin1(kind) + "-" + QString::fromStdString(face) + "-" +
           QString::fromStdString(wire);
}

template <class Source> QJsonObject wireFeature(const char *kind, const Source &source)
{
    QJsonArray faces;
    for (const auto &id : source.sourceFaceIds)
        faces.append(QString::fromStdString(id));
    return {{"kind", kind}, {"id", featureId(kind, source.rootFaceId, source.rootWireId)}, {"faces", faces}};
}
} // namespace

QJsonArray planningFeatures(const ThreeAxisWireTraceUseCaseResult &trace)
{
    QJsonArray features;
    for (const auto &s : trace.pocketSources)
        features.append(wireFeature("pocket", s));
    for (const auto &s : trace.profileSources)
        features.append(wireFeature("profile", s));
    for (const auto f : {MoldSetupFace::PositiveZ, MoldSetupFace::NegativeZ, MoldSetupFace::PositiveX,
                         MoldSetupFace::NegativeX, MoldSetupFace::PositiveY, MoldSetupFace::NegativeY})
        features.append(QJsonObject{{"kind", "engrave"},
                                    {"id", QString("engrave-") + moldSetupFaceName(f)},
                                    {"faces", QJsonArray{}}});
    return features;
}

void planFeature(MachiningJob &job, const QJsonObject &request, const Model &model, const StepIndexer &index,
                 const tooling::ToolLibrary &library, const toolpath::MoldSetupWorkFrames &frames,
                 const TopoDS_Shape &part)
{
    const auto feature = request["feature"].toString();
    const auto toolId = request["tool"].toString();
    const auto *bit = library.toolById(toolId);
    const auto *slot = library.slotForTool(toolId);
    if (!bit || !slot || !tooling::hasSupportedMillingGeometry(*bit))
        throw std::runtime_error("请选择有有效刀位的铣刀");
    const auto tool = tooling::makeToolpathToolContext({*bit, *slot});
    const double step = request["step"].toDouble(1);
    if (!(step > 0 && step <= bit->geometry.cuttingEdgeLength))
        throw std::runtime_error("步距或层深超出刀具范围");
    const auto libraryId = library.id().toStdString();
    std::string error;
    if (feature.startsWith("engrave-"))
    {
        for (const auto face : {MoldSetupFace::PositiveZ, MoldSetupFace::NegativeZ, MoldSetupFace::PositiveX,
                                MoldSetupFace::NegativeX, MoldSetupFace::PositiveY, MoldSetupFace::NegativeY})
        {
            if (feature != QString("engrave-") + moldSetupFaceName(face))
                continue;
            const auto *frame = frames.forFace(face);
            if (!frame || !frame->valid)
                throw std::runtime_error("无效雕刻装夹");
            toolpath::SurfaceFinishingParams p;
            p.cutterKind = bit->type == tooling::ToolType::BallEndMill ? toolpath::FinishingCutterKind::Ball
                           : bit->type == tooling::ToolType::BullNose  ? toolpath::FinishingCutterKind::Bull
                                                                       : toolpath::FinishingCutterKind::Flat;
            p.diameter = tool.diameter;
            p.cuttingLength = tool.cuttingEdgeLength;
            p.cornerRadius = tool.cornerRadius;
            p.stepOver = step;
            p.sampleSpacing = std::min(0.5, step);
            p.feed = tool.horizontalFeed;
            p.plungeFeed = tool.verticalFeed;
            SurfaceFinishingJobCaptureRequest capture;
            capture.setupFrames = &frames;
            capture.setupFace = face;
            capture.tool = &tool;
            capture.libraryId = libraryId;
            std::vector<TopoDS_Shape> targets;
            for (const auto &u : model.units)
            {
                if (const auto *shape = index.shapeForUnit(u.unit_id))
                {
                    targets.push_back(*shape);
                    capture.unitIds.push_back(u.unit_id);
                }
            }
            TopoDS_Shape stock;
            if (request["strategy"].toString() == "rough")
            {
                p.mode = toolpath::EngravingMachiningMode::StockRoughingAndFinishing;
                stock = SurfaceFinishingMachiningJobService::buildPlanningStock(job.definition().stock,
                                                                                targets, &error);
                if (stock.IsNull())
                    throw std::runtime_error(error);
            }
            const auto result = toolpath::SurfaceFinishing().generate(part, frame->setupWorkFrame, p, nullptr,
                                                                      stock.IsNull() ? nullptr : &stock);
            if (!result.ok)
                throw std::runtime_error(result.error);
            capture.result = &result;
            if (!SurfaceFinishingMachiningJobService().capture(job, capture, &error))
                throw std::runtime_error(error);
            return;
        }
        throw std::runtime_error("未找到雕刻装夹");
    }
    const auto holes = FeatureRecognizer().recognizeHoleGeometries(model, index);
    const auto trace = traceRegions(model, index, holes.geometries);
    if (!trace.ok())
        throw std::runtime_error(trace.error);
    tooling::ToolLibrary single(library.id(), library.name());
    if (!single.upsertTool(*bit) || !single.assignToolToSlot(*slot))
        throw std::runtime_error("刀位无效");
    const auto frame = MoldCoordinateFrame::fromModel(model);
    for (const auto &s : trace.pocketSources)
    {
        if (feature != featureId("pocket", s.rootFaceId, s.rootWireId))
            continue;
        toolpath::RootInnerWirePocketSource source{s.kind,
                                                   s.sourceFaceIds,
                                                   s.rootFaceId,
                                                   s.rootWireId,
                                                   s.allowOpenSectionContours,
                                                   s.openBoundaryFaceCount};
        const auto regions = toolpath::PocketRegionBuilder().buildRootInnerWireRegions({source}, index);
        if (regions.empty())
            throw std::runtime_error("无法建立清腔区域");
        auto p = job.definition().pocketParameters;
        p.depth.stepDown = step;
        const auto strategy = request["strategy"].toString();
        if (strategy == "zigzag")
            p.clearing.pattern = toolpath::ClearingPattern::ZigZag;
        else if (strategy == "spiral")
            p.clearing.pattern = toolpath::ClearingPattern::Spiral;
        else if (strategy == "freepath")
            p.clearing.pattern = toolpath::ClearingPattern::FreePath;
        else
            p.clearing.pattern = toolpath::ClearingPattern::Offset;
        auto planned = tooling::PocketAutomaticToolpath().generate(single, regions, p);
        if (!planned.ok())
            throw std::runtime_error(planned.error.empty() ? "清腔规划失败" : planned.error);
        PocketTargetReference target;
        target.sourceKind = PocketRegionSourceKind::RootInnerWire;
        target.kind = s.kind;
        target.sourceFaceIds = s.sourceFaceIds;
        target.rootFaceId = s.rootFaceId;
        target.rootWireId = s.rootWireId;
        target.allowOpenSectionContours = s.allowOpenSectionContours;
        target.openBoundaryFaceCount = s.openBoundaryFaceCount;
        target.floorFaceId = regions.front().floorFaceId;
        for (auto &op : planned.batch.operations)
        {
            op.setupFace = classifyMoldSetupFace(frame, gp_Vec(regions.front().openDir));
            op.setupIndex = static_cast<int>(op.setupFace);
        }
        std::vector<PocketTargetReference> targets(planned.batch.operations.size(), target);
        PocketJobCaptureRequest capture{&planned.batch.operations, &targets, &planned.batch.effectiveParams,
                                        &frames, libraryId};
        if (!PocketMachiningJobService().capture(job, capture, &error))
            throw std::runtime_error(error);
        return;
    }
    for (const auto &s : trace.profileSources)
    {
        if (feature != featureId("profile", s.rootFaceId, s.rootWireId))
            continue;
        toolpath::RootOuterWireProfileSource source{
            s.unitId,        s.axisLabel,        s.normalDirection, s.rootFaceId,      s.rootWireId,
            s.sourceFaceIds, s.machiningFaceIds, s.terminalFaceIds, s.mainOuterFaceIds};
        toolpath::ProfileRegionBuildOptions opts;
        const auto &stock = job.definition().stock;
        opts.stockMarginX = stock.marginX;
        opts.stockMarginY = stock.marginY;
        opts.stockMarginZ = stock.marginZ;
        const auto regions = toolpath::ProfileRegionBuilder().build({source}, index, opts);
        if (regions.empty())
            throw std::runtime_error("无法建立外轮廓区域");
        auto p = job.definition().profileParameters;
        p.depth.stepDown = step;
        p.profile.machiningMode = request["strategy"].toString() == "contour"
                                      ? toolpath::ProfileMachiningMode::ContourOnly
                                      : toolpath::ProfileMachiningMode::ExternalClearing;
        tooling::ProfileAutomaticToolpathResult planned;
        if (p.profile.machiningMode == toolpath::ProfileMachiningMode::ContourOnly)
        {
            const auto effective = toolpath::bindPocketToolParameters(p, tool);
            planned.batch = toolpath::ProfileToolpathService().generate(regions, effective, "PROFILE_");
            planned.validation = planned.batch.safetyValidation;
            if (!planned.batch.safetyValidationPerformed)
                throw std::runtime_error("轮廓安全验证未完成");
            if (!planned.batch.failures.empty())
                throw std::runtime_error(planned.batch.failures.front().contour.error);
            for (auto &op : planned.batch.operations)
            {
                op.tool = tool;
                op.finishPass = effective.profile.finishPass;
                op.restMachiningPreviousToolDiameter = effective.clearing.restMachiningPreviousToolDiameter;
            }
        }
        else
            planned = tooling::ProfileAutomaticToolpath().generate(single, regions, p);
        if (!planned.ok())
            throw std::runtime_error(planned.error.empty() ? "外轮廓规划失败" : planned.error);
        ProfileTargetReference target;
        target.axisLabel = s.axisLabel;
        target.normalDirection = s.normalDirection;
        target.sourceFaceIds = s.sourceFaceIds;
        target.machiningFaceIds = s.machiningFaceIds;
        target.terminalFaceIds = s.terminalFaceIds;
        target.mainOuterFaceIds = s.mainOuterFaceIds;
        target.rootFaceId = s.rootFaceId;
        target.rootWireId = s.rootWireId;
        for (auto &op : planned.batch.operations)
        {
            op.setupFace = classifyMoldSetupFace(frame, gp_Vec(regions.front().openDir));
            op.setupIndex = static_cast<int>(op.setupFace);
        }
        std::vector<ProfileTargetReference> targets(planned.batch.operations.size(), target);
        ProfileJobCaptureRequest capture{&planned.batch.operations, &targets, &planned.batch.effectiveParams,
                                         &frames, libraryId};
        if (!ProfileMachiningJobService().capture(job, capture, &error))
            throw std::runtime_error(error);
        return;
    }
    throw std::runtime_error("在此模型中找不到所选加工区域");
}
} // namespace oap::worker
