// RecognitionGeometryCache 实现（oap_recognition）。
// 逐体几何缓存：主外表面判定、面包围盒、边—面映射、边质心与有效平面等
// 查询的构建与缓存，见 RecognitionGeometryCache.h。
#include "RecognitionGeometryCache.h"

#include "AAG.h"
#include "analysis/SurfaceClassify.h"
#include "analysis/AabbBoundaryFace.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp_Dir.hxx>

namespace oap {

bool RecognitionGeometryCache::build(
    const TopoDS_Shape& shape,
    const AAG& graph,
    RecognitionGeometryCacheMode mode,
    const RecognitionGeometryCacheControl* control)
{
    *this = RecognitionGeometryCache{};
    RecognitionGeometryCache pending;
    if (!pending.buildImpl(shape, graph, nullptr, nullptr, mode, control))
        return false;
    *this = std::move(pending);
    return true;
}

bool RecognitionGeometryCache::build(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const Bnd_Box& shapeBox,
    const std::vector<Bnd_Box>& faceBoxes,
    RecognitionGeometryCacheMode mode,
    const RecognitionGeometryCacheControl* control)
{
    *this = RecognitionGeometryCache{};
    RecognitionGeometryCache pending;
    if (!pending.buildImpl(shape, graph, &shapeBox, &faceBoxes, mode, control))
        return false;
    *this = std::move(pending);
    return true;
}

// 统一实现：面盒和有效平面供两种模式的主外表面判定共享；Full 模式
// 额外构建边—面映射、面邻接与并行预计算的线质心。
bool RecognitionGeometryCache::buildImpl(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const Bnd_Box* suppliedShapeBox,
    const std::vector<Bnd_Box>* suppliedFaceBoxes,
    RecognitionGeometryCacheMode mode,
    const RecognitionGeometryCacheControl* control)
{
    std::atomic_bool cancellationObserved{false};
    const auto cancelled = [&] {
        if (cancellationObserved.load(std::memory_order_relaxed))
            return true;
        if (control && control->isCancellationRequested &&
            control->isCancellationRequested()) {
            cancellationObserved.store(true, std::memory_order_relaxed);
            return true;
        }
        return false;
    };
    const auto executeRange = [&](std::size_t count,
                                  const std::function<void(std::size_t)>& task) {
        if (control && control->executeRange) {
            control->executeRange(count, task);
        } else {
            for (std::size_t index = 0; index < count && !cancelled(); ++index)
                task(index);
        }
    };
    m_shapeBox.SetVoid();
    m_faceBoxes.clear();
    m_mainOuterFaces.clear();
    m_edgeFaces.Clear();
    m_faceNeighbors.clear();
    m_faceBoundaryEdgeIndices.clear();
    m_edgeCenters.clear();
    m_edgeCenterStates.clear();
    m_faces.clear();
    m_effectivePlanes.clear();
    m_effectivePlaneStates.clear();

    const int faceCount = graph.faceMap.Extent();
    if (shape.IsNull() || faceCount == 0 || cancelled())
        return false;

    if (suppliedShapeBox && !suppliedShapeBox->IsVoid()) {
        m_shapeBox = *suppliedShapeBox;
    } else {
        try {
            BRepBndLib::AddOptimal(
                shape, m_shapeBox,
                Standard_False, Standard_False);
        } catch (const Standard_Failure&) {
            m_shapeBox.SetVoid();
        }
    }

    const bool accumulateShapeBox = m_shapeBox.IsVoid();
    m_faceBoxes.resize(static_cast<std::size_t>(faceCount + 1));
    m_faces.resize(static_cast<std::size_t>(faceCount + 1));
    m_effectivePlanes.resize(static_cast<std::size_t>(faceCount + 1));
    m_effectivePlaneStates.assign(static_cast<std::size_t>(faceCount + 1), 0);
    m_mainOuterFaces.assign(
        static_cast<std::size_t>(faceCount + 1), false);
    const bool buildFullCache =
        mode ==
        RecognitionGeometryCacheMode::Full;
    if (buildFullCache) {
        m_faceNeighbors.resize(
            static_cast<std::size_t>(
                faceCount + 1));
        m_faceBoundaryEdgeIndices.resize(
            static_cast<std::size_t>(
                faceCount + 1));
    }

    for (int faceIndex = 1;
         faceIndex <= faceCount; ++faceIndex) {
        if (cancelled())
            return false;
        m_faces[static_cast<std::size_t>(faceIndex)] =
            TopoDS::Face(graph.faceMap(faceIndex));
        Bnd_Box& faceBox =
            m_faceBoxes[static_cast<std::size_t>(faceIndex)];
        if (suppliedFaceBoxes &&
            static_cast<std::size_t>(faceIndex) <
                suppliedFaceBoxes->size() &&
            !(*suppliedFaceBoxes)
                 [static_cast<std::size_t>(faceIndex)]
                     .IsVoid()) {
            faceBox = (*suppliedFaceBoxes)
                [static_cast<std::size_t>(faceIndex)];
        } else {
            try {
                BRepBndLib::AddOptimal(
                    graph.faceMap(faceIndex), faceBox,
                    Standard_False, Standard_False);
            } catch (const Standard_Failure&) {
                faceBox.SetVoid();
            }
        }

        if (accumulateShapeBox && !faceBox.IsVoid())
            m_shapeBox.Add(faceBox);
    }

    // 两种模式均只分类一次；主外表面判定和后续追踪共享此结果。
    // 沿用调用方执行预算，未注入执行器时在当前线程运行。
    executeRange(static_cast<std::size_t>(faceCount),
        [&](std::size_t index) {
            if (cancelled())
                return;
            const std::size_t faceIndex = index + 1u;
            try {
                OCC_CATCH_SIGNALS
                thread_local bool signalsInitialized = false;
                if (!signalsInitialized) {
                    OSD::SetThreadLocalSignal(
                        OSD_SignalMode_Set, Standard_False);
                    signalsInitialized = true;
                }
                gp_Pln classifiedPlane;
                if (oap::effectivePlane(
                        m_faces[static_cast<std::size_t>(faceIndex)],
                        classifiedPlane)) {
                    m_effectivePlanes[
                        static_cast<std::size_t>(faceIndex)] =
                        classifiedPlane;
                    m_effectivePlaneStates[
                        static_cast<std::size_t>(faceIndex)] = 1;
                }
            } catch (const Standard_Failure&) {
                m_effectivePlaneStates[
                    static_cast<std::size_t>(faceIndex)] = 0;
            } catch (...) {
                m_effectivePlaneStates[
                    static_cast<std::size_t>(faceIndex)] = 0;
            }
        });

    if (cancelled())
        return false;

    // 主外表面以“裁剪后仍贴合 AABB 边界”为判据：先用轴向平面构造参考盒，
    // 无轴向平面时回退到整体紧致包围盒，再逐个检查候选面的边界贴合度。
    Bnd_Box mainOuterReferenceBox;
    for (int faceIndex = 1; faceIndex <= faceCount; ++faceIndex) {
        if (cancelled())
            return false;
        const auto index = static_cast<std::size_t>(faceIndex);
        if (m_effectivePlaneStates[index] == 1)
            addAxisAlignedPlanarFaceBox(m_effectivePlanes[index],
                                       m_faceBoxes[index], mainOuterReferenceBox);
    }
    if (mainOuterReferenceBox.IsVoid())
        mainOuterReferenceBox = m_shapeBox;

    if (!mainOuterReferenceBox.IsVoid()) {
        for (int faceIndex = 1;
             faceIndex <= faceCount; ++faceIndex) {
            if (cancelled())
                return false;
            const Bnd_Box& faceBox =
                m_faceBoxes[static_cast<std::size_t>(faceIndex)];
            if (m_effectivePlaneStates[static_cast<std::size_t>(faceIndex)] != 1 ||
                faceBox.IsVoid() ||
                mainOuterReferenceBox.IsOut(faceBox))
                continue;
            try {
                m_mainOuterFaces[
                    static_cast<std::size_t>(faceIndex)] =
                    isFaceOnAabbBoundary(
                        m_effectivePlanes[static_cast<std::size_t>(faceIndex)],
                        mainOuterReferenceBox, faceBox);
            } catch (const Standard_Failure&) {
                m_mainOuterFaces[
                    static_cast<std::size_t>(faceIndex)] = false;
            }
        }
    }

    if (cancelled())
        return false;
    if (!buildFullCache)
        return true;

    TopExp::MapShapesAndAncestors(
        shape, TopAbs_EDGE, TopAbs_FACE, m_edgeFaces);
    m_edgeCenters.resize(
        static_cast<std::size_t>(
            m_edgeFaces.Extent() + 1));
    m_edgeCenterStates.assign(
        static_cast<std::size_t>(
            m_edgeFaces.Extent() + 1),
        static_cast<signed char>(0));

    // 记录每张面的去重边下标，同时建立包含非流形边在内的完整面邻接。
    for (int edgeIndex = 1;
         edgeIndex <= m_edgeFaces.Extent(); ++edgeIndex) {
        if (cancelled())
            return false;
        std::vector<int> ancestorFaceIndices;
        const TopTools_ListOfShape& ancestorFaces =
            m_edgeFaces.FindFromIndex(edgeIndex);
        for (TopTools_ListIteratorOfListOfShape iterator(
                 ancestorFaces);
             iterator.More(); iterator.Next()) {
            if (iterator.Value().ShapeType() != TopAbs_FACE)
                continue;
            const int faceIndex =
                graph.faceMap.FindIndex(iterator.Value());
            if (faceIndex >= 1 && faceIndex <= faceCount)
                ancestorFaceIndices.push_back(faceIndex);
        }
        std::sort(
            ancestorFaceIndices.begin(),
            ancestorFaceIndices.end());
        ancestorFaceIndices.erase(
            std::unique(
                ancestorFaceIndices.begin(),
                ancestorFaceIndices.end()),
            ancestorFaceIndices.end());

        for (const int first : ancestorFaceIndices) {
            std::vector<int>& neighbors =
                m_faceNeighbors[
                    static_cast<std::size_t>(first)];
            m_faceBoundaryEdgeIndices[
                static_cast<std::size_t>(first)]
                    .push_back(edgeIndex);
            for (const int second : ancestorFaceIndices) {
                if (first != second)
                    neighbors.push_back(second);
            }
        }
    }
    for (std::vector<int>& neighbors : m_faceNeighbors) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(
            std::unique(neighbors.begin(), neighbors.end()),
            neighbors.end());
    }

    const int edgeCount = m_edgeFaces.Extent();
    std::vector<TopoDS_Edge> edges(
        static_cast<std::size_t>(edgeCount + 1));
    for (int edgeIndex = 1; edgeIndex <= edgeCount; ++edgeIndex) {
        if (cancelled())
            return false;
        edges[static_cast<std::size_t>(edgeIndex)] =
            TopoDS::Edge(m_edgeFaces.FindKey(edgeIndex));
    }
    executeRange(static_cast<std::size_t>(edgeCount),
        [&](std::size_t index) {
            if (cancelled())
                return;
            const std::size_t edgeIndex = index + 1u;
            try {
                OCC_CATCH_SIGNALS
                thread_local bool signalsInitialized = false;
                if (!signalsInitialized) {
                    OSD::SetThreadLocalSignal(
                        OSD_SignalMode_Set, Standard_False);
                    signalsInitialized = true;
                }
                const TopoDS_Edge& edge =
                    edges[static_cast<std::size_t>(edgeIndex)];
                if (edge.IsNull() || BRep_Tool::Degenerated(edge))
                    return;

                GProp_GProps properties;
                BRepGProp::LinearProperties(edge, properties);
                if (!std::isfinite(properties.Mass()) ||
                    properties.Mass() <= Precision::Confusion()) {
                    return;
                }
                const gp_Pnt center = properties.CentreOfMass();
                if (!std::isfinite(center.X()) ||
                    !std::isfinite(center.Y()) ||
                    !std::isfinite(center.Z())) {
                    return;
                }
                m_edgeCenters[
                    static_cast<std::size_t>(edgeIndex)] = center;
                m_edgeCenterStates[
                    static_cast<std::size_t>(edgeIndex)] = 1;
            } catch (const Standard_Failure&) {
                m_edgeCenterStates[
                    static_cast<std::size_t>(edgeIndex)] = 0;
            } catch (...) {
                m_edgeCenterStates[
                    static_cast<std::size_t>(edgeIndex)] = 0;
            }
        });
    return !cancelled();
}

bool RecognitionGeometryCache::isMainOuterFace(
    int faceIndex) const
{
    return faceIndex >= 1 &&
           static_cast<std::size_t>(faceIndex) <
               m_mainOuterFaces.size() &&
           m_mainOuterFaces[
               static_cast<std::size_t>(faceIndex)];
}

const Bnd_Box* RecognitionGeometryCache::faceBox(
    int faceIndex) const
{
    if (faceIndex < 1 ||
        static_cast<std::size_t>(faceIndex) >=
            m_faceBoxes.size())
        return nullptr;
    return &m_faceBoxes[
        static_cast<std::size_t>(faceIndex)];
}

bool RecognitionGeometryCache::faceBoxCenter(
    int faceIndex,
    gp_Pnt& center) const
{
    const Bnd_Box* box = faceBox(faceIndex);
    if (!box || box->IsVoid())
        return false;

    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    box->Get(
        xMin, yMin, zMin, xMax, yMax, zMax);
    center.SetCoord(
        0.5 * (xMin + xMax),
        0.5 * (yMin + yMax),
        0.5 * (zMin + zMax));
    return std::isfinite(center.X()) &&
           std::isfinite(center.Y()) &&
           std::isfinite(center.Z());
}

const std::vector<int>& RecognitionGeometryCache::faceNeighbors(
    int faceIndex) const
{
    static const std::vector<int> kEmpty;
    if (faceIndex < 1 ||
        static_cast<std::size_t>(faceIndex) >=
            m_faceNeighbors.size()) {
        return kEmpty;
    }
    return m_faceNeighbors[
        static_cast<std::size_t>(faceIndex)];
}

bool RecognitionGeometryCache::faceBoundaryEdgeProjectionRange(
    int faceIndex,
    const gp_Dir& axisDirection,
    double& minimum,
    double& maximum) const
{
    if (faceIndex < 1 ||
        static_cast<std::size_t>(faceIndex) >=
            m_faceBoundaryEdgeIndices.size()) {
        return false;
    }
    const std::vector<int>& edgeIndices =
        m_faceBoundaryEdgeIndices[
            static_cast<std::size_t>(faceIndex)];
    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (const int edgeIndex : edgeIndices) {
        if (edgeIndex < 1 ||
            static_cast<std::size_t>(edgeIndex) >=
                m_edgeCenterStates.size()) {
            continue;
        }
        const signed char state =
            m_edgeCenterStates[
                static_cast<std::size_t>(edgeIndex)];
        if (state != 1)
            continue;
        const gp_Pnt& center =
            m_edgeCenters[
                static_cast<std::size_t>(edgeIndex)];
        const double height =
            center.X() * axisDirection.X() +
            center.Y() * axisDirection.Y() +
            center.Z() * axisDirection.Z();
        if (!std::isfinite(height))
            continue;
        minimum = std::min(minimum, height);
        maximum = std::max(maximum, height);
    }
    return std::isfinite(minimum) &&
           std::isfinite(maximum);
}

bool RecognitionGeometryCache::effectivePlane(
    int faceIndex,
    gp_Pln& plane) const
{
    if (faceIndex < 1 ||
        static_cast<std::size_t>(faceIndex) >=
            m_effectivePlaneStates.size()) {
        return false;
    }
    const signed char state =
        m_effectivePlaneStates[
            static_cast<std::size_t>(faceIndex)];
    if (state != 1)
        return false;
    plane = m_effectivePlanes[
        static_cast<std::size_t>(faceIndex)];
    return true;
}

} // namespace oap
