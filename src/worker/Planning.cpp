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
    const bool automatic = request["autoTool"].toBool();
    if (automatic && (feature.startsWith("engrave-") ||
                      (feature.startsWith("profile-") && request["strategy"] == "contour")))
        throw std::runtime_error("此策略需要指定铣刀");
    if (!automatic && (!bit || !slot || !tooling::hasSupportedMillingGeometry(*bit)))
        throw std::runtime_error("请选择有有效刀位的铣刀");
    const auto tool =
        automatic ? toolpath::ToolpathToolContext{} : tooling::makeToolpathToolContext({*bit, *slot});
    const double step = request["step"].toDouble(1);
    if (!(step > 0 && (automatic || step <= bit->geometry.cuttingEdgeLength)))
        throw std::runtime_error("步距或层深超出刀具范围");
    const auto libraryId = library.id().toStdString();
    std::string error;
    if (feature.startsWith("engrave-"))
    {
        const bool allFaces = feature == "engrave-all";
        std::vector<toolpath::SurfaceFinishingResult> results;
        results.reserve(6);
        std::vector<SurfaceFinishingJobCaptureRequest> captures;
        for (const auto face : {MoldSetupFace::PositiveZ, MoldSetupFace::NegativeZ, MoldSetupFace::PositiveX,
                                MoldSetupFace::NegativeX, MoldSetupFace::PositiveY, MoldSetupFace::NegativeY})
        {
            if (!allFaces && feature != QString("engrave-") + moldSetupFaceName(face))
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
                stock = SurfaceFinishingMachiningJobService::buildPlanningStock(
                    job.definition().stock, targets, &error, nullptr,
                    request["stockFile"].toString().toStdString());
                if (stock.IsNull())
                    throw std::runtime_error(error);
            }
            auto result = toolpath::SurfaceFinishing().generate(part, frame->setupWorkFrame, p, nullptr,
                                                                stock.IsNull() ? nullptr : &stock);
            if (!result.ok)
                throw std::runtime_error(result.error);
            if (allFaces)
            {
                results.push_back(std::move(result));
                capture.result = &results.back();
                captures.push_back(std::move(capture));
                continue;
            }
            capture.result = &result;
            if (!SurfaceFinishingMachiningJobService().capture(job, capture, &error))
                throw std::runtime_error(error);
            return;
        }
        if (!allFaces)
            throw std::runtime_error("未找到雕刻装夹");
        if (!SurfaceFinishingMachiningJobService().replaceFamily(job, captures, &error))
            throw std::runtime_error(error);
        return;
    }
    const auto holes = FeatureRecognizer().recognizeHoleGeometries(model, index);
    const auto trace = traceRegions(model, index, holes.geometries);
    if (!trace.ok())
        throw std::runtime_error(trace.error);
    tooling::ToolLibrary single(library.id(), library.name());
    if (!automatic && (!single.upsertTool(*bit) || !single.assignToolToSlot(*slot)))
        throw std::runtime_error("刀位无效");
    const auto &planningLibrary = automatic ? library : single;
    const auto frame = MoldCoordinateFrame::fromModel(model);
    const auto capturePocket = [&](const std::vector<toolpath::PocketRegion> &regions,
                                   const std::vector<PocketTargetReference> &sources) {
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
        auto planned = tooling::PocketAutomaticToolpath().generate(planningLibrary, regions, p);
        if (!planned.ok())
            throw std::runtime_error(planned.error.empty() ? "清腔规划失败" : planned.error);
        std::vector<PocketTargetReference> targets;
        for (auto &op : planned.batch.operations)
        {
            const auto sourceIndex = static_cast<std::size_t>(op.sourceRegionIndex);
            targets.push_back(sources.at(sourceIndex));
            targets.back().floorFaceId = regions.at(sourceIndex).floorFaceId;
            op.setupFace = classifyMoldSetupFace(frame, gp_Vec(regions.at(sourceIndex).openDir));
            op.setupIndex = static_cast<int>(op.setupFace);
        }
        PocketJobCaptureRequest capture{&planned.batch.operations, &targets, &planned.batch.effectiveParams,
                                        &frames, libraryId};
        capture.append = feature != "pocket-all";
        if (!PocketMachiningJobService().capture(job, capture, &error))
            throw std::runtime_error(error);
    };
    if (feature.startsWith("manual-pocket-") || feature.startsWith("manual-slot-"))
    {
        FeatureInstance instance;
        instance.manual = true;
        instance.kind = feature.startsWith("manual-slot-") ? FeatureKind::Slot : FeatureKind::Pocket;
        instance.floorFaceId = feature.section('-', 2).toStdString();
        instance.faceIds =
            toolpath::PocketRegionBuilder::expandManualFloorSeed(instance.floorFaceId, model, &error);
        if (instance.faceIds.empty())
            throw std::runtime_error(error);
        const auto region = toolpath::PocketRegionBuilder().buildInstance(instance, index, {}, &error);
        if (!region)
            throw std::runtime_error(error);
        PocketTargetReference target;
        target.kind = instance.kind;
        target.sourceFaceIds = instance.faceIds;
        capturePocket({*region}, {std::move(target)});
        return;
    }
    std::vector<toolpath::PocketRegion> pockets;
    std::vector<PocketTargetReference> pocketTargets;
    for (const auto &s : trace.pocketSources)
    {
        if (feature != "pocket-all" && feature != featureId("pocket", s.rootFaceId, s.rootWireId))
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
        PocketTargetReference target;
        target.sourceKind = PocketRegionSourceKind::RootInnerWire;
        target.kind = s.kind;
        target.sourceFaceIds = s.sourceFaceIds;
        target.rootFaceId = s.rootFaceId;
        target.rootWireId = s.rootWireId;
        target.allowOpenSectionContours = s.allowOpenSectionContours;
        target.openBoundaryFaceCount = s.openBoundaryFaceCount;
        pockets.push_back(regions.front());
        pocketTargets.push_back(std::move(target));
    }
    if (!pockets.empty())
    {
        capturePocket(pockets, pocketTargets);
        return;
    }
    std::vector<toolpath::ProfileRegion> profiles;
    std::vector<ProfileTargetReference> sources;
    for (const auto &s : trace.profileSources)
    {
        if (feature != "profile-all" && feature != featureId("profile", s.rootFaceId, s.rootWireId))
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
        ProfileTargetReference target;
        target.axisLabel = s.axisLabel;
        target.normalDirection = s.normalDirection;
        target.sourceFaceIds = s.sourceFaceIds;
        target.machiningFaceIds = s.machiningFaceIds;
        target.terminalFaceIds = s.terminalFaceIds;
        target.mainOuterFaceIds = s.mainOuterFaceIds;
        target.rootFaceId = s.rootFaceId;
        target.rootWireId = s.rootWireId;
        profiles.push_back(regions.front());
        sources.push_back(std::move(target));
    }
    if (!profiles.empty())
    {
        auto p = job.definition().profileParameters;
        p.depth.stepDown = step;
        p.profile.machiningMode = request["strategy"].toString() == "contour"
                                      ? toolpath::ProfileMachiningMode::ContourOnly
                                      : toolpath::ProfileMachiningMode::ExternalClearing;
        tooling::ProfileAutomaticToolpathResult planned;
        if (p.profile.machiningMode == toolpath::ProfileMachiningMode::ContourOnly)
        {
            const auto effective = toolpath::bindPocketToolParameters(p, tool);
            planned.batch = toolpath::ProfileToolpathService().generate(profiles, effective, "PROFILE_");
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
            planned = tooling::ProfileAutomaticToolpath().generate(planningLibrary, profiles, p);
        if (!planned.ok())
            throw std::runtime_error(planned.error.empty() ? "外轮廓规划失败" : planned.error);
        std::vector<ProfileTargetReference> targets;
        for (auto &op : planned.batch.operations)
        {
            const auto sourceIndex = static_cast<std::size_t>(op.sourceRegionIndex);
            targets.push_back(sources.at(sourceIndex));
            op.setupFace = classifyMoldSetupFace(frame, gp_Vec(profiles.at(sourceIndex).openDir));
            op.setupIndex = static_cast<int>(op.setupFace);
        }
        ProfileJobCaptureRequest capture{&planned.batch.operations, &targets, &planned.batch.effectiveParams,
                                         &frames, libraryId};
        if (!ProfileMachiningJobService().capture(job, capture, &error))
            throw std::runtime_error(error);
        return;
    }
    throw std::runtime_error("在此模型中找不到所选加工区域");
}
} // namespace oap::worker
