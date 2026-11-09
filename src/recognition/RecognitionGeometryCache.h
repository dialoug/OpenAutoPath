// 识别阶段的逐体几何缓存。主外表面、紧致包围盒和边—面映射在同一 Unit
// 内由通结构、轴向平面筛选以及 Z/Y/X 内外环追踪共同复用。
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <Bnd_Box.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>

class AAG;
class TopoDS_Shape;
class gp_Dir;

namespace oap {

// Executes every index in [0, count) once, and joins all work before returning
// or rethrowing. The callback and its captures must not escape this call.
// The caller supplies admission policy; recognition has no thread-pool dependency.
using RecognitionRangeExecutor = std::function<void(
    std::size_t count, const std::function<void(std::size_t)>& task)>;

struct RecognitionGeometryCacheControl {
    // May be called concurrently by an injected executor.
    std::function<bool()> isCancellationRequested;
    RecognitionRangeExecutor executeRange;
};

// 两种模式均缓存面包围盒、有效平面和主外表面标记；Full 额外构建
// 边—面拓扑与边质心，供通结构及内外环追踪复用。
enum class RecognitionGeometryCacheMode {
    Full,
    MainOuterFacesOnly
};

// 属于 oap_recognition 模块。逐体构建、以 const 只读方式交给各识别器复用；
// build() 返回后成员不再修改，可在并行预计算完成后的多线程只读场景共享。
// 构建期间仅修改临时缓存；失败、取消或异常时本对象为空。未提供执行回调时串行。
class RecognitionGeometryCache {
public:
    // 独立算法入口使用：从 OCCT 几何计算紧致包围盒。
    bool build(
        const TopoDS_Shape& shape,
        const AAG& graph,
        RecognitionGeometryCacheMode mode =
            RecognitionGeometryCacheMode::Full,
        const RecognitionGeometryCacheControl* control = nullptr);

    // 已完成 STEP 索引时使用：复用 Model 中已经计算好的紧致包围盒。
    // faceBoxes 采用与 AAG faceMap 相同的 1-based 索引，元素 0 不使用；
    // 缺失或无效项会自动回退到 OCCT 计算。
    bool build(const TopoDS_Shape& shape,
               const AAG& graph,
               const Bnd_Box& shapeBox,
               const std::vector<Bnd_Box>& faceBoxes,
               RecognitionGeometryCacheMode mode =
                   RecognitionGeometryCacheMode::Full,
               const RecognitionGeometryCacheControl* control = nullptr);

    // 判定面是否为逐体 AABB 主外表面；越界索引返回 false。
    bool isMainOuterFace(int faceIndex) const;
    // 返回面的紧致包围盒指针；索引越界或面盒无效时返回 nullptr。
    const Bnd_Box* faceBox(int faceIndex) const;
    // 整个体的紧致包围盒；build() 失败或几何异常时为 Void 盒。
    const Bnd_Box& shapeBox() const { return m_shapeBox; }
    // 共享边 -> 邻接面映射（TopExp::MapShapesAndAncestors 结果），只读复用。
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaces() const {
        return m_edgeFaces;
    }
    // 逐体构建一次的共享边邻接，供三轴追踪、分支拆分和末级 BFS 复用。
    const std::vector<int>& faceNeighbors(int faceIndex) const;

    // 每条拓扑边的线质心在构建阶段并行预计算。查询时只读并按任意轴投影，
    // 返回当前面的最低/最高边线质心高度，避免根面追踪期间写共享缓存。
    bool faceBoundaryEdgeProjectionRange(
        int faceIndex,
        const gp_Dir& axisDirection,
        double& minimum,
        double& maximum) const;

    // 返回已缓存紧致包围盒的中心，供面位置、相对高度和等高判定统一使用。
    bool faceBoxCenter(int faceIndex, gp_Pnt& center) const;
    // 有效平面（含一般曲面的平面回归）在构建阶段并行预计算；XYZ 平面筛选、
    // 终止面和槽腔末级法向判定只读并共用同一结果。
    bool effectivePlane(int faceIndex, gp_Pln& plane) const;

private:
    bool buildImpl(const TopoDS_Shape& shape,
                   const AAG& graph,
                   const Bnd_Box* suppliedShapeBox,
                   const std::vector<Bnd_Box>* suppliedFaceBoxes,
                   RecognitionGeometryCacheMode mode,
                   const RecognitionGeometryCacheControl* control);

    Bnd_Box m_shapeBox;
    std::vector<Bnd_Box> m_faceBoxes;
    std::vector<bool> m_mainOuterFaces;
    TopTools_IndexedDataMapOfShapeListOfShape m_edgeFaces;
    std::vector<std::vector<int>> m_faceNeighbors;
    std::vector<std::vector<int>> m_faceBoundaryEdgeIndices;
    std::vector<gp_Pnt> m_edgeCenters;
    // 0=无有效线质心，1=有效；build() 返回后不再修改。
    std::vector<signed char> m_edgeCenterStates;
    std::vector<TopoDS_Face> m_faces;
    std::vector<gp_Pln> m_effectivePlanes;
    // 0=非有效平面，1=有效平面；build() 返回后不再修改。
    std::vector<signed char> m_effectivePlaneStates;
};

} // namespace oap
