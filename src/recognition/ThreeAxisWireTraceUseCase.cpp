// 三轴识别同步用例：集中 AAG、追踪、排他归属和盲腔连通合并。
#include "recognition/ThreeAxisWireTraceUseCase.h"

#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "recognition/AAG.h"
#include "recognition/AxisParallelPlaneRecognizer.h"
#include "recognition/ConnectedBlindPocketMerger.h"
#include "recognition/RecognitionGeometryCache.h"
#include "recognition/ThroughStructureRecognizer.h"

#include <Bnd_Box.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iterator>
#include <optional>
#include <unordered_map>
#include <utility>

namespace oap {
namespace {

// 轴向规格：标签 + 单位方向。顺序固定为 Z、Y、X，决定统计数组下标。
struct AxisSpec {
    const char* label = "";
    gp_Dir direction;
};

// 单条根面追踪结果的临时记录，携带排序、排他与合并所需的全部派生信息。
struct TraceRecord {
    int axisOrder = 0;
    std::string unitId;
    std::string axisLabel;
    std::string rootFaceId;
    std::string rootWireId;
    bool fromInnerWire = true;
    bool rootIsMainOuterFace = false;
    int normalDirection = 1;
    double rootHeight = 0.0;
    AxisWireTraceFeatureKind featureKind =
        AxisWireTraceFeatureKind::BlindPocket;
    bool rootCanConnectBlindPockets = false;
    std::string stableSortKey;
    std::vector<std::string> faceIds;
    std::vector<std::string> terminalFaceIds;
};

Bnd_Box toOcctBox(const Bbox& box)
{
    Bnd_Box result;
    if (box.valid()) {
        result.Update(
            box.xmin, box.ymin, box.zmin,
            box.xmax, box.ymax, box.zmax);
    }
    return result;
}

// 复用 STEP 索引阶段已计算的面/体包围盒构建几何缓存，避免重复 OCCT 计算；
// 面盒与 AAG faceMap 同为 1-based 索引，缺失项由缓存内部回退计算。
bool buildGeometryCache(
    const TopoDS_Shape& shape,
    const Unit& unit,
    const AAG& graph,
    const StepIndexer& indexer,
    const ModelIndex& modelIndex,
    RecognitionGeometryCache& cache,
    const ThreeAxisWireTraceControl* control)
{
    std::vector<Bnd_Box> faceBoxes(
        static_cast<std::size_t>(graph.faceMap.Extent() + 1));
    for (int faceIndex = 1;
         faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
        if (control && control->cancelled())
            return false;
        const std::string faceId = indexer.idForFace(
            TopoDS::Face(graph.faceMap(faceIndex)));
        const Face* face = modelIndex.face(faceId);
        if (face && face->bbox.valid()) {
            faceBoxes[static_cast<std::size_t>(faceIndex)] =
                toOcctBox(face->bbox);
        }
    }
    RecognitionGeometryCacheControl cacheControl;
    if (control) {
        cacheControl.isCancellationRequested = control->isCancellationRequested;
        cacheControl.executeRange = control->executeRange;
    }
    return cache.build(shape, graph, toOcctBox(unit.bbox), faceBoxes,
                       RecognitionGeometryCacheMode::Full, &cacheControl);
}

bool cancelled(const ThreeAxisWireTraceControl* control)
{
    return control && control->cancelled();
}

void sortUnique(std::vector<std::string>& values)
{
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

} // namespace

ThreeAxisWireTraceUseCaseResult ThreeAxisWireTraceUseCase::execute(
    const ThreeAxisWireTraceRequest& request,
    const ThreeAxisWireTraceControl* control) const
{
    ThreeAxisWireTraceUseCaseResult output;
    // 固定按 Z/Y/X 顺序处理，与统计数组 axisStats 的下标一一对应。
    const std::array<AxisSpec, 3> axes{{
        {"Z", gp::DZ()},
        {"Y", gp::DY()},
        {"X", gp::DX()},
    }};
    for (std::size_t index = 0; index < axes.size(); ++index)
        output.axisStats[index].axisLabel = axes[index].label;

    if (!request.model || !request.modelIndex || !request.stepIndexer) {
        output.error = "three-axis recognition requires model, index and topology maps";
        return output;
    }
    const Model& model = *request.model;
    const ModelIndex& modelIndex = *request.modelIndex;
    const StepIndexer& stepIndexer = *request.stepIndexer;
    if (model.faces.empty() || model.units.empty()) {
        output.error = "three-axis recognition requires an analyzed model";
        return output;
    }

    std::vector<TraceRecord> traceRecords;
    std::vector<ThreeAxisProfileSource> profileSourceCandidates;
    if (control)
        control->progress(0, model.units.size());

    try {
        OCC_CATCH_SIGNALS
        OSD::SetThreadLocalSignal(
            OSD_SignalMode_Set, Standard_False);
        for (std::size_t unitIndex = 0;
             unitIndex < model.units.size(); ++unitIndex) {
            if (cancelled(control))
                break;
            const Unit& unit = model.units[unitIndex];
            const TopoDS_Shape* shape =
                stepIndexer.shapeForUnit(unit.unit_id);
            if (!shape || shape->IsNull()) {
                if (control)
                    control->progress(unitIndex + 1, model.units.size());
                continue;
            }

            try {
                // 每个 Unit 独立构建 AAG 与几何缓存；同一 Unit 内所有识别器
                // 共用同一缓存与累计排除集。
                AAG graph;
                graph.build(*shape);
                RecognitionGeometryCache geometryCache;
                if (!buildGeometryCache(
                    *shape, unit, graph, stepIndexer,
                    modelIndex, geometryCache, control)) {
                    if (cancelled(control))
                        break;
                    ++output.failedUnitCount;
                    if (control)
                        control->progress(unitIndex + 1, model.units.size());
                    continue;
                }
                // 把已识别的孔面（按 STEP id）映射为 AAG 索引，作为后续
                // 通结构/平面筛选/追踪的全局排除集合。
                std::vector<std::string> faceIdsByIndex(
                    static_cast<std::size_t>(graph.faceMap.Extent() + 1));
                std::unordered_set<int> excludedIndices;
                for (int faceIndex = 1;
                     faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
                    std::string& faceId = faceIdsByIndex[
                        static_cast<std::size_t>(faceIndex)];
                    faceId = stepIndexer.idForFace(
                        TopoDS::Face(graph.faceMap(faceIndex)));
                    if (!faceId.empty() &&
                        request.recognizedHoleFaceIds.contains(faceId)) {
                        excludedIndices.insert(faceIndex);
                    }
                }

                const auto faceIdForIndex =
                    [&](int faceIndex) -> const std::string& {
                        static const std::string empty;
                        if (faceIndex < 1 ||
                            faceIndex > graph.faceMap.Extent()) {
                            return empty;
                        }
                        return faceIdsByIndex[
                            static_cast<std::size_t>(faceIndex)];
                    };
                std::unordered_set<std::string> mainOuterFaceIds;
                for (int faceIndex = 1;
                     faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
                    if (geometryCache.isMainOuterFace(faceIndex)) {
                        const std::string& faceId = faceIdForIndex(faceIndex);
                        if (!faceId.empty())
                            mainOuterFaceIds.insert(faceId);
                    }
                }
                // 从根面的内环线中挑出与分支首层相邻边共享最多的那条，
                // 作为该分支对应的根内环；无匹配时返回空串。
                const auto rootWireIdForBranch =
                    [&](int rootFaceIndex,
                        const AxisWireTraceBranch& branch,
                        const char* wireRole) {
                        const std::string& rootFaceId =
                            faceIdForIndex(rootFaceIndex);
                        const Face* rootRecord =
                            modelIndex.face(rootFaceId);
                        if (!rootRecord)
                            return std::string{};

                        std::unordered_set<std::string> firstLevelFaceIds;
                        for (const int faceIndex :
                             branch.firstLevelFaceIndices) {
                            const std::string& faceId =
                                faceIdForIndex(faceIndex);
                            if (!faceId.empty())
                                firstLevelFaceIds.insert(faceId);
                        }
                        if (firstLevelFaceIds.empty())
                            return std::string{};

                        std::string bestWireId;
                        int bestSharedEdgeCount = 0;
                        for (const std::string& wireId :
                             rootRecord->wire_ids) {
                            const Wire* wire = modelIndex.wire(wireId);
                            if (!wire || wire->wire_role != wireRole)
                                continue;
                            int sharedEdgeCount = 0;
                            for (const std::string& edgeId : wire->edge_ids) {
                                const Edge* edge = modelIndex.edge(edgeId);
                                if (!edge)
                                    continue;
                                const bool reachesFirstLevel = std::any_of(
                                    edge->face_ids.begin(), edge->face_ids.end(),
                                    [&](const std::string& adjacentFaceId) {
                                        return firstLevelFaceIds.contains(
                                            adjacentFaceId);
                                    });
                                if (reachesFirstLevel)
                                    ++sharedEdgeCount;
                            }
                            if (sharedEdgeCount > bestSharedEdgeCount) {
                                bestSharedEdgeCount = sharedEdgeCount;
                                bestWireId = wireId;
                            }
                        }
                        return bestWireId;
                    };
                const auto rootOuterWireId = [&](int rootFaceIndex) {
                    const Face* rootRecord = modelIndex.face(
                        faceIdForIndex(rootFaceIndex));
                    if (!rootRecord)
                        return std::string{};
                    for (const std::string& wireId : rootRecord->wire_ids) {
                        const Wire* wire = modelIndex.wire(wireId);
                        if (wire && wire->wire_role == "outer" &&
                            wire->is_closed) {
                            return wireId;
                        }
                    }
                    return std::string{};
                };

                try {
                    // 先做通结构识别：其面加入排除集，后续平面筛选与追踪不再
                    // 重复归属。单 Unit 的异常不阻断整个三轴流程。
                    const ThroughStructureRecognitionResult through =
                        RecognizeThroughStructuresOCCT(
                            *shape, graph, excludedIndices, geometryCache);
                    output.verticalSurfaceCount += through.verticalSurfaceCount;
                    output.excludedHoleFaceCount +=
                        through.excludedHoleFaceCount;
                    output.excludedMainOuterFaceCount +=
                        through.excludedMainOuterFaceCount;
                    output.rejectedWithoutNonZNeighborCount +=
                        through.rejectedWithoutNonZNeighborCount;
                    output.rejectedNonConvexCount +=
                        through.rejectedNonConvexCount;
                    for (const int faceIndex : through.faceIndices) {
                        excludedIndices.insert(faceIndex);
                        const std::string& faceId =
                            faceIdForIndex(faceIndex);
                        if (!faceId.empty())
                            output.throughStructureFaceIds.insert(faceId);
                    }
                } catch (const Standard_Failure&) {
                    // 通结构异常不阻断同一实体的三轴追踪。
                } catch (const std::exception&) {
                    // 同上。
                }

                for (std::size_t axisIndex = 0;
                     axisIndex < axes.size(); ++axisIndex) {
                    if (cancelled(control))
                        break;
                    const AxisSpec& axis = axes[axisIndex];
                    ThreeAxisTraceStats& stats =
                        output.axisStats[axisIndex];
                    const AxisParallelPlaneRecognitionResult planes =
                        RecognizeAxisParallelPlanesOCCT(
                            *shape, graph, axis.direction,
                            excludedIndices, geometryCache);
                    stats.skippedInvalidFaceCount +=
                        planes.skippedInvalidFaceCount;
                    for (const int faceIndex : planes.faceIndices) {
                        const std::string& faceId =
                            faceIdForIndex(faceIndex);
                        if (faceId.empty())
                            continue;
                        stats.planeFaceIds.insert(faceId);
                        output.axisParallelPlaneFaceIds.insert(faceId);
                    }

                    const CombinedWireTraceResult traced =
                        TraceCombinedWireLoopsAlongAxisOCCT(
                            *shape, graph, axis.direction,
                            excludedIndices, geometryCache);
                    stats.rootFaceCount += traced.rootFaceCount;
                    stats.skippedInvalidFaceCount +=
                        traced.skippedInvalidFaceCount;
                    stats.terminalFaceCount += static_cast<int>(
                        traced.terminalFaceIndices.size());

                    for (const AxisWireTracePath& path : traced.paths) {
                        // 主外平面的 outer wire 本身就是 Profile 的自动来源。
                        // 即使相邻外侧壁被主外表面规则阻止进入追踪路径，也可以
                        // 从线框共享边直接取得首层侧壁，避免简单长方体漏识别。
                        if (geometryCache.isMainOuterFace(path.rootFaceIndex)) {
                            ThreeAxisProfileSource profile;
                            profile.unitId = unit.unit_id;
                            profile.axisLabel = axis.label;
                            profile.normalDirection = path.normalDirection;
                            profile.rootFaceId =
                                faceIdForIndex(path.rootFaceIndex);
                            profile.rootWireId =
                                rootOuterWireId(path.rootFaceIndex);
                            profile.mainOuterFaceIds.assign(
                                mainOuterFaceIds.begin(),
                                mainOuterFaceIds.end());
                            sortUnique(profile.mainOuterFaceIds);
                            const Wire* wire =
                                modelIndex.wire(profile.rootWireId);
                            if (wire) {
                                for (const std::string& edgeId : wire->edge_ids) {
                                    const Edge* edge = modelIndex.edge(edgeId);
                                    if (!edge)
                                        continue;
                                    for (const std::string& faceId : edge->face_ids) {
                                        if (!faceId.empty() &&
                                            faceId != profile.rootFaceId) {
                                            profile.sourceFaceIds.push_back(faceId);
                                        }
                                    }
                                }
                            }
                            sortUnique(profile.sourceFaceIds);
                            if (!profile.rootWireId.empty() &&
                                !profile.sourceFaceIds.empty()) {
                                profileSourceCandidates.push_back(
                                    std::move(profile));
                            }
                        }
                        const auto appendBranch =
                            [&](const AxisWireTraceBranch& branch,
                                int innerBranchIndex,
                                const std::string& branchSortKey) {
                                // 把一条分支（内环或外环）折叠为 TraceRecord：
                                // 根面为外主表面且为内环时翻转法向，保证朝向一致。
                                TraceRecord record;
                                record.axisOrder =
                                    static_cast<int>(axisIndex);
                                record.unitId = unit.unit_id;
                                record.axisLabel = axis.label;
                                record.rootFaceId =
                                    faceIdForIndex(path.rootFaceIndex);
                                record.fromInnerWire = innerBranchIndex >= 0;
                                record.rootIsMainOuterFace =
                                    geometryCache.isMainOuterFace(
                                        path.rootFaceIndex);
                                if (record.fromInnerWire) {
                                    record.rootWireId =
                                        rootWireIdForBranch(
                                            path.rootFaceIndex, branch,
                                            "inner");
                                } else {
                                    record.rootWireId =
                                        rootWireIdForBranch(
                                            path.rootFaceIndex, branch,
                                            "outer");
                                }
                                record.normalDirection = path.normalDirection;
                                if (record.fromInnerWire &&
                                    record.rootIsMainOuterFace) {
                                    record.normalDirection *= -1;
                                }
                                record.rootHeight = path.rootHeight;
                                record.featureKind = branch.featureKind;
                                record.rootCanConnectBlindPockets =
                                    !record.rootIsMainOuterFace;
                                record.stableSortKey =
                                    unit.unit_id + ":" +
                                    std::to_string(path.rootFaceIndex) +
                                    ":" + branchSortKey;
                                for (const int faceIndex :
                                     branch.traversedFaceIndices) {
                                    const std::string& faceId =
                                        faceIdForIndex(faceIndex);
                                    if (faceId.empty())
                                        continue;
                                    record.faceIds.push_back(faceId);
                                    stats.traceFaceIds.insert(faceId);
                                    output.combinedTraceFaceIds.insert(faceId);
                                }
                                for (const int faceIndex :
                                     branch.terminalFaceIndices) {
                                    const std::string& faceId =
                                        faceIdForIndex(faceIndex);
                                    if (!faceId.empty())
                                        record.terminalFaceIds.push_back(faceId);
                                }
                                sortUnique(record.faceIds);
                                sortUnique(record.terminalFaceIds);
                                if (record.faceIds.empty())
                                    return;
                                ++stats.activePathCount;
                                if (branch.featureKind ==
                                    AxisWireTraceFeatureKind::BlindSlot) {
                                    ++stats.blindSlotPathCount;
                                } else {
                                    ++stats.blindPocketPathCount;
                                }
                                traceRecords.push_back(std::move(record));
                            };
                        for (std::size_t index = 0;
                             index < path.outerTraces.size(); ++index) {
                            appendBranch(
                                path.outerTraces[index], -1,
                                "0-outer-" + std::to_string(index));
                        }
                        for (std::size_t index = 0;
                             index < path.innerTraces.size(); ++index) {
                            appendBranch(
                                path.innerTraces[index],
                                static_cast<int>(index),
                                "1-inner-" + std::to_string(index));
                        }
                    }

                    excludedIndices.insert(
                        planes.faceIndices.begin(), planes.faceIndices.end());
                    excludedIndices.insert(
                        traced.traversedFaceIndices.begin(),
                        traced.traversedFaceIndices.end());
                    for (const AxisWireTracePath& path : traced.paths) {
                        if (path.rootFaceIndex > 0)
                            excludedIndices.insert(path.rootFaceIndex);
                    }
                }
            } catch (const Standard_Failure&) {
                ++output.failedUnitCount;
            } catch (const std::exception&) {
                ++output.failedUnitCount;
            } catch (...) {
                ++output.failedUnitCount;
            }
            if (control)
                control->progress(unitIndex + 1, model.units.size());
        }
    } catch (const Standard_Failure& failure) {
        const char* message = failure.GetMessageString();
        output.error = message ? message : "OCCT three-axis recognition failure";
        return output;
    } catch (const std::exception& failure) {
        output.error = failure.what();
        return output;
    } catch (...) {
        output.error = "unknown three-axis recognition failure";
        return output;
    }

    output.cancelled = cancelled(control);
    if (output.cancelled)
        return output;

    // 稳定排序：按轴向、法向、根面高度、最后到稳定键排序，保证多 Unit/多轴
    // 之间输出顺序确定，排他归属与合并均基于该顺序。
    std::sort(
        traceRecords.begin(), traceRecords.end(),
        [](const TraceRecord& first, const TraceRecord& second) {
            if (first.axisOrder != second.axisOrder)
                return first.axisOrder < second.axisOrder;
            if (first.normalDirection != second.normalDirection)
                return first.normalDirection > second.normalDirection;
            if (std::abs(first.rootHeight - second.rootHeight) > 1.0e-6) {
                return first.normalDirection > 0
                    ? first.rootHeight < second.rootHeight
                    : first.rootHeight > second.rootHeight;
            }
            return first.stableSortKey < second.stableSortKey;
        });

    // 排他归属：同一面被多条记录共享时，排序靠后的记录接管该面，并累计覆盖计数。
    std::unordered_map<std::string, std::size_t> finalOwnerByFace;
    for (std::size_t recordIndex = 0;
         recordIndex < traceRecords.size(); ++recordIndex) {
        for (const std::string& faceId : traceRecords[recordIndex].faceIds) {
            auto [owner, inserted] =
                finalOwnerByFace.emplace(faceId, recordIndex);
            if (!inserted) {
                owner->second = recordIndex;
                ++output.overwrittenFaceCount;
            }
        }
    }

    std::vector<ThreeAxisFeatureSubset> rawSubsets;
    rawSubsets.reserve(traceRecords.size());
    for (std::size_t recordIndex = 0;
         recordIndex < traceRecords.size(); ++recordIndex) {
        const TraceRecord& record = traceRecords[recordIndex];
        std::vector<std::string> ownedFaceIds;
        for (const std::string& faceId : record.faceIds) {
            const auto owner = finalOwnerByFace.find(faceId);
            if (owner != finalOwnerByFace.end() &&
                owner->second == recordIndex) {
                ownedFaceIds.push_back(faceId);
            }
        }
        if (ownedFaceIds.empty())
            continue;
        ThreeAxisFeatureSubset subset;
        subset.axisLabel = record.axisLabel;
        subset.rootFaceId = record.rootFaceId;
        subset.rootWireId = record.rootWireId;
        subset.fromInnerWire = record.fromInnerWire;
        subset.rootIsMainOuterFace = record.rootIsMainOuterFace;
        subset.featureKind = record.featureKind;
        subset.faceIds = std::move(ownedFaceIds);
        subset.rootCanConnectBlindPockets =
            record.rootCanConnectBlindPockets;
        subset.terminalFaceIds = record.terminalFaceIds;
        rawSubsets.push_back(std::move(subset));
    }

    std::unordered_map<std::string, std::size_t> pocketSourceByRootWire;
    std::vector<std::size_t> pocketSourceRepresentativeRecordIndices;
    for (std::size_t recordIndex = 0;
         recordIndex < traceRecords.size(); ++recordIndex) {
        const TraceRecord& record = traceRecords[recordIndex];
        if (!record.fromInnerWire || !record.rootIsMainOuterFace)
            continue;
        ThreeAxisPocketSource source;
        source.kind = record.featureKind ==
                AxisWireTraceFeatureKind::BlindSlot
            ? FeatureKind::Slot : FeatureKind::Pocket;
        source.sourceFaceIds = record.faceIds;
        source.rootFaceId = record.rootFaceId;
        source.rootWireId = record.rootWireId;
        if (source.rootWireId.empty()) {
            output.pocketSources.push_back(std::move(source));
            pocketSourceRepresentativeRecordIndices.push_back(recordIndex);
            continue;
        }
        const std::string key =
            source.rootFaceId + "\x1f" + source.rootWireId;
        const auto [position, inserted] = pocketSourceByRootWire.emplace(
            key, output.pocketSources.size());
        if (inserted) {
            output.pocketSources.push_back(std::move(source));
            pocketSourceRepresentativeRecordIndices.push_back(recordIndex);
            continue;
        }
        ThreeAxisPocketSource& retained =
            output.pocketSources[position->second];
        retained.sourceFaceIds.insert(
            retained.sourceFaceIds.end(),
            source.sourceFaceIds.begin(), source.sourceFaceIds.end());
        if (source.kind == FeatureKind::Pocket) {
            retained.kind = FeatureKind::Pocket;
            pocketSourceRepresentativeRecordIndices[position->second] =
                recordIndex;
        }
    }
    for (ThreeAxisPocketSource& source : output.pocketSources)
        sortUnique(source.sourceFaceIds);

    // 与 FreeCAD Profile 的“未选择几何时使用整个模型外轮廓”语义一致：
    // 主外表面的 outer wire 自动成为外轮廓源。按根面/根线框去重，避免同一
    // 装夹方向的追踪分支重复生成相同轮廓。
    std::unordered_map<std::string, std::size_t> profileSourceByRootWire;
    const auto mergeProfileSource =
        [&](ThreeAxisProfileSource source) {
            const std::string key = source.unitId + "\x1f" +
                source.rootFaceId + "\x1f" + source.rootWireId;
            const auto [position, inserted] =
                profileSourceByRootWire.emplace(
                    key, output.profileSources.size());
            if (inserted) {
                output.profileSources.push_back(std::move(source));
                return;
            }
            ThreeAxisProfileSource& retained =
                output.profileSources[position->second];
            retained.sourceFaceIds.insert(
                retained.sourceFaceIds.end(),
                source.sourceFaceIds.begin(), source.sourceFaceIds.end());
            retained.machiningFaceIds.insert(
                retained.machiningFaceIds.end(),
                source.machiningFaceIds.begin(), source.machiningFaceIds.end());
            retained.terminalFaceIds.insert(
                retained.terminalFaceIds.end(),
                source.terminalFaceIds.begin(), source.terminalFaceIds.end());
            retained.mainOuterFaceIds.insert(
                retained.mainOuterFaceIds.end(),
                source.mainOuterFaceIds.begin(), source.mainOuterFaceIds.end());
        };
    for (ThreeAxisProfileSource& source : profileSourceCandidates)
        mergeProfileSource(std::move(source));
    for (const TraceRecord& record : traceRecords) {
        if (record.fromInnerWire || !record.rootIsMainOuterFace ||
            record.rootFaceId.empty() || record.rootWireId.empty()) {
            continue;
        }
        const Wire* wire = modelIndex.wire(record.rootWireId);
        if (!wire || wire->wire_role != "outer" || !wire->is_closed)
            continue;

        ThreeAxisProfileSource source;
        source.unitId = record.unitId;
        source.axisLabel = record.axisLabel;
        source.normalDirection = record.normalDirection;
        source.rootFaceId = record.rootFaceId;
        source.rootWireId = record.rootWireId;
        source.sourceFaceIds = record.faceIds;
        source.machiningFaceIds = record.faceIds;
        source.terminalFaceIds = record.terminalFaceIds;
        mergeProfileSource(std::move(source));
    }
    for (ThreeAxisProfileSource& source : output.profileSources) {
        sortUnique(source.sourceFaceIds);
        sortUnique(source.machiningFaceIds);
        sortUnique(source.terminalFaceIds);
        sortUnique(source.mainOuterFaceIds);
    }
    // 非主外表面的 outer-wire 分支也可能是主轮廓向内扫描时遇到的局部台阶，
    // 例如 DIE 样例中 F44 根分支通过 F38/F41/F45 与 F5 的外轮廓链相连。
    // 这些分支不能作为独立装夹面，但必须并入同轴、同开口方向且已经连通的
    // Profile；递归合并允许连续多个台阶逐级扩展。局部根平面一并作为加工面，
    // 使深度能够准确到达该台阶，而不是停在上一个追踪分支的最低边。
    std::vector<bool> attachedOuterTrace(traceRecords.size(), false);
    bool attachedAnyOuterTrace = false;
    do {
        attachedAnyOuterTrace = false;
        for (std::size_t recordIndex = 0;
             recordIndex < traceRecords.size(); ++recordIndex) {
            if (attachedOuterTrace[recordIndex])
                continue;
            const TraceRecord& record = traceRecords[recordIndex];
            if (record.fromInnerWire || record.rootIsMainOuterFace ||
                record.faceIds.empty()) {
                continue;
            }

            std::size_t bestSourceIndex = output.profileSources.size();
            std::size_t bestSharedFaceCount = 0;
            for (std::size_t sourceIndex = 0;
                 sourceIndex < output.profileSources.size(); ++sourceIndex) {
                const ThreeAxisProfileSource& source =
                    output.profileSources[sourceIndex];
                if (source.unitId != record.unitId ||
                    source.axisLabel != record.axisLabel ||
                    source.normalDirection != record.normalDirection) {
                    continue;
                }
                std::size_t sharedFaceCount = 0;
                for (const std::string& faceId : record.faceIds) {
                    if (std::binary_search(
                            source.sourceFaceIds.begin(),
                            source.sourceFaceIds.end(), faceId)) {
                        ++sharedFaceCount;
                    }
                }
                if (sharedFaceCount > bestSharedFaceCount) {
                    bestSharedFaceCount = sharedFaceCount;
                    bestSourceIndex = sourceIndex;
                }
            }
            if (bestSourceIndex == output.profileSources.size())
                continue;

            ThreeAxisProfileSource& target =
                output.profileSources[bestSourceIndex];
            target.sourceFaceIds.insert(
                target.sourceFaceIds.end(),
                record.faceIds.begin(), record.faceIds.end());
            target.sourceFaceIds.push_back(record.rootFaceId);
            target.machiningFaceIds.insert(
                target.machiningFaceIds.end(),
                record.faceIds.begin(), record.faceIds.end());
            target.machiningFaceIds.push_back(record.rootFaceId);
            target.terminalFaceIds.insert(
                target.terminalFaceIds.end(),
                record.terminalFaceIds.begin(), record.terminalFaceIds.end());
            sortUnique(target.sourceFaceIds);
            sortUnique(target.machiningFaceIds);
            sortUnique(target.terminalFaceIds);
            attachedOuterTrace[recordIndex] = true;
            attachedAnyOuterTrace = true;
        }
    } while (attachedAnyOuterTrace);

    // 相反装夹面的 Profile 可以共享贯穿的基础外壁。它们仍保留在
    // sourceFaceIds 中用于构造闭合截面，但不应把两侧加工深度同时拉到整件
    // 厚度；当前方向的 machiningFaceIds 只保留本方向独有的特征面。
    for (std::size_t firstIndex = 0;
         firstIndex < output.profileSources.size(); ++firstIndex) {
        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < output.profileSources.size(); ++secondIndex) {
            ThreeAxisProfileSource& first = output.profileSources[firstIndex];
            ThreeAxisProfileSource& second = output.profileSources[secondIndex];
            if (first.unitId != second.unitId ||
                first.axisLabel != second.axisLabel ||
                first.normalDirection == second.normalDirection) {
                continue;
            }
            std::vector<std::string> shared;
            std::set_intersection(
                first.sourceFaceIds.begin(), first.sourceFaceIds.end(),
                second.sourceFaceIds.begin(), second.sourceFaceIds.end(),
                std::back_inserter(shared));
            if (shared.empty())
                continue;
            const auto uniqueMachiningFaces = [&](ThreeAxisProfileSource& value) {
                std::vector<std::string> unique;
                std::set_difference(
                    value.sourceFaceIds.begin(), value.sourceFaceIds.end(),
                    shared.begin(), shared.end(),
                    std::back_inserter(unique));
                if (!unique.empty())
                    value.machiningFaceIds = std::move(unique);
            };
            uniqueMachiningFaces(first);
            uniqueMachiningFaces(second);
        }
    }
    // 盲腔连通合并：以“可连接”根面/终端面作为连接面，把多条子集并成组，
    // 再与孔源匹配，让属于同一腔的开口共享同一特征。
    std::vector<BlindPocketMergeRegion> mergeRegions;
    mergeRegions.reserve(rawSubsets.size());
    for (const ThreeAxisFeatureSubset& subset : rawSubsets) {
        BlindPocketMergeRegion region;
        region.featureKind = subset.featureKind;
        region.faceIds = subset.faceIds;
        if (subset.rootCanConnectBlindPockets &&
            !subset.rootFaceId.empty()) {
            region.connectorFaceIds.push_back(subset.rootFaceId);
        }
        if (subset.rootCanConnectBlindPockets &&
            !subset.fromInnerWire) {
            region.connectorFaceIds.insert(
                region.connectorFaceIds.end(),
                subset.terminalFaceIds.begin(),
                subset.terminalFaceIds.end());
        }
        sortUnique(region.connectorFaceIds);
        mergeRegions.push_back(std::move(region));
    }
    const auto connectedGroups =
        GroupConnectedBlindPocketRegions(mergeRegions, modelIndex);

    std::vector<std::vector<std::string>> pocketSourceFaceIds;
    pocketSourceFaceIds.reserve(output.pocketSources.size());
    for (const ThreeAxisPocketSource& source : output.pocketSources)
        pocketSourceFaceIds.push_back(source.sourceFaceIds);
    const BlindPocketSourceGroupMatches pocketMatches =
        MatchBlindPocketSourcesToConnectedGroups(
            connectedGroups, mergeRegions, pocketSourceFaceIds);

    for (std::size_t groupIndex = 0;
         groupIndex < connectedGroups.size(); ++groupIndex) {
        std::vector<std::string> outerFeatureFaces;
        for (const std::size_t sourceIndex : connectedGroups[groupIndex]) {
            if (sourceIndex < rawSubsets.size() &&
                !rawSubsets[sourceIndex].fromInnerWire) {
                outerFeatureFaces.insert(
                    outerFeatureFaces.end(),
                    rawSubsets[sourceIndex].faceIds.begin(),
                    rawSubsets[sourceIndex].faceIds.end());
            }
        }
        const auto& targets = pocketMatches.sourceIndicesByGroup[groupIndex];
        if (outerFeatureFaces.empty() || targets.empty())
            continue;
        for (const std::size_t target : targets) {
            ThreeAxisPocketSource& source = output.pocketSources[target];
            source.sourceFaceIds.insert(
                source.sourceFaceIds.end(),
                outerFeatureFaces.begin(), outerFeatureFaces.end());
        }
    }
    for (ThreeAxisPocketSource& source : output.pocketSources)
        sortUnique(source.sourceFaceIds);

    // 对每个孔源做沿边 BFS：把通过公共边连通的通结构面并入该源，并标记
    // 允许开口截面轮廓（由通结构提供侧面轮廓）。
    std::unordered_map<std::string, std::vector<std::string>>
        throughFaceIdsByEdge;
    throughFaceIdsByEdge.reserve(
        output.throughStructureFaceIds.size() * 4);
    for (const std::string& throughFaceId :
         output.throughStructureFaceIds) {
        const Face* face = modelIndex.face(throughFaceId);
        if (!face)
            continue;
        for (const std::string& edgeId : face->edge_ids) {
            if (!edgeId.empty())
                throughFaceIdsByEdge[edgeId].push_back(throughFaceId);
        }
    }
    for (ThreeAxisPocketSource& source : output.pocketSources) {
        std::unordered_set<std::string> connectedThroughFaces;
        std::vector<std::string> pending;
        const auto enqueue = [&](const std::string& faceId) {
            if (!output.throughStructureFaceIds.contains(faceId) ||
                !connectedThroughFaces.insert(faceId).second) {
                return;
            }
            pending.push_back(faceId);
        };
        for (const std::string& sourceFaceId : source.sourceFaceIds) {
            enqueue(sourceFaceId);
            const Face* face = modelIndex.face(sourceFaceId);
            if (!face)
                continue;
            for (const std::string& edgeId : face->edge_ids) {
                const auto adjacent = throughFaceIdsByEdge.find(edgeId);
                if (adjacent == throughFaceIdsByEdge.end())
                    continue;
                for (const std::string& faceId : adjacent->second)
                    enqueue(faceId);
            }
        }
        for (std::size_t index = 0; index < pending.size(); ++index) {
            const Face* face = modelIndex.face(pending[index]);
            if (!face)
                continue;
            for (const std::string& edgeId : face->edge_ids) {
                const auto adjacent = throughFaceIdsByEdge.find(edgeId);
                if (adjacent == throughFaceIdsByEdge.end())
                    continue;
                for (const std::string& faceId : adjacent->second)
                    enqueue(faceId);
            }
        }
        if (connectedThroughFaces.empty())
            continue;
        source.sourceFaceIds.insert(
            source.sourceFaceIds.end(),
            connectedThroughFaces.begin(), connectedThroughFaces.end());
        source.allowOpenSectionContours = true;
        source.openBoundaryFaceCount =
            static_cast<int>(connectedThroughFaces.size());
        sortUnique(source.sourceFaceIds);
    }

    // 把每个连通组合并为一个 ThreeAxisFeatureSubset；组内优先取孔源代表
    // 记录的面，使合并结果保留入口/根面语义。
    output.featureSubsets.reserve(connectedGroups.size());
    for (std::size_t groupIndex = 0;
         groupIndex < connectedGroups.size(); ++groupIndex) {
        const auto& group = connectedGroups[groupIndex];
        if (group.empty())
            continue;
        ThreeAxisFeatureSubset merged = rawSubsets[group.back()];
        const std::optional<std::size_t>& preferred =
            pocketMatches.preferredSourceIndexByGroup[groupIndex];
        if (merged.featureKind == AxisWireTraceFeatureKind::BlindPocket &&
            preferred &&
            *preferred < pocketSourceRepresentativeRecordIndices.size()) {
            const std::size_t recordIndex =
                pocketSourceRepresentativeRecordIndices[*preferred];
            if (recordIndex < traceRecords.size()) {
                const TraceRecord& representative = traceRecords[recordIndex];
                merged.axisLabel = representative.axisLabel;
                merged.rootFaceId = representative.rootFaceId;
                merged.rootWireId = representative.rootWireId;
                merged.fromInnerWire = true;
                merged.rootIsMainOuterFace = true;
                merged.rootCanConnectBlindPockets = false;
                merged.terminalFaceIds = representative.terminalFaceIds;
            }
        }
        merged.faceIds.clear();
        for (const std::size_t sourceIndex : group) {
            merged.faceIds.insert(
                merged.faceIds.end(),
                rawSubsets[sourceIndex].faceIds.begin(),
                rawSubsets[sourceIndex].faceIds.end());
        }
        sortUnique(merged.faceIds);
        merged.mergedSourceCount = group.size();
        if (merged.featureKind == AxisWireTraceFeatureKind::BlindPocket &&
            group.size() > 1) {
            output.absorbedBlindPocketSubsetCount +=
                static_cast<int>(group.size() - 1);
        }
        output.featureSubsets.push_back(std::move(merged));
    }
    return output;
}

} // namespace oap
