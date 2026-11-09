// 构建面邻接属性图（AAG），缓存共享边和拓扑邻接供各识别器复用。
// 模块: oap_recognition。AAG 的完整数据结构见 AAG.h；本文件实现 build() 与
//       sharedEdge()，仅依赖 OCCT 拓扑遍历，不依赖 oap_analysis。
#include "AAG.h"

#include <algorithm>  // std::find（邻接去重）
#include <cstdint>

#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

namespace
{
    // 流形邻接边：恰好被两个面共享的边才登记进邻接图
    constexpr int kManifoldFaceCount = 2;
}

// 无序面下标对键：较小者放高 32 位、较大者放低 32 位，与面顺序无关。
std::uint64_t AAG::facePairKey(
    int firstFaceIndex,
    int secondFaceIndex)
{
    const auto first = static_cast<std::uint32_t>(
        std::min(firstFaceIndex, secondFaceIndex));
    const auto second = static_cast<std::uint32_t>(
        std::max(firstFaceIndex, secondFaceIndex));
    return (static_cast<std::uint64_t>(first) << 32U) |
           static_cast<std::uint64_t>(second);
}

const TopoDS_Edge* AAG::sharedEdge(
    int firstFaceIndex,
    int secondFaceIndex) const
{
    if (firstFaceIndex < 1 ||
        secondFaceIndex < 1 ||
        firstFaceIndex == secondFaceIndex) {
        return nullptr;
    }
    const auto found = m_sharedEdges.find(
        facePairKey(firstFaceIndex, secondFaceIndex));
    return found == m_sharedEdges.end()
               ? nullptr
               : &found->second;
}

// 构建 AAG 邻接图：建立面索引与边-面映射，遍历两流形共享边建立面邻接关系。
void AAG::build(const TopoDS_Shape& shape)
{
    faceMap.Clear();
    adjacency.clear();
    m_sharedEdges.clear();

    // 1. 建立 Face 索引
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
    adjacency.reserve(
        static_cast<std::size_t>(
            faceMap.Extent()));

    // 2. Edge -> Faces 映射
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(shape,
                                 TopAbs_EDGE,
                                 TopAbs_FACE,
                                 edgeToFaces);
    m_sharedEdges.reserve(
        static_cast<std::size_t>(
            edgeToFaces.Extent()));

    // 3. 遍历所有 Edge
    for (int i = 1; i <= edgeToFaces.Extent(); ++i)
    {
        const TopTools_ListOfShape& faces =
            edgeToFaces.FindFromIndex(i);

        // 只处理 2 个面的情况（正常邻接）
        if (faces.Extent() != kManifoldFaceCount)
            continue;

        auto it = faces.begin();
        const TopoDS_Face& f1 = TopoDS::Face(*it++);
        const TopoDS_Face& f2 = TopoDS::Face(*it);

        int id1 = faceMap.FindIndex(f1);
        int id2 = faceMap.FindIndex(f2);

        // 防自环:周期面(圆柱/锥/球/环)的缝合边在同一面的 wire 里正反各用一次,
        // 非去重的 MapShapesAndAncestors 会把同一面登记两次 -> id1==id2。
        if (id1 < 1 || id2 < 1 ||
            id1 == id2)
            continue;

        const TopoDS_Edge edge =
            TopoDS::Edge(edgeToFaces.FindKey(i));
        if (!edge.IsNull() &&
            !BRep_Tool::Degenerated(edge)) {
            m_sharedEdges.try_emplace(
                facePairKey(id1, id2), edge);
        }

        // 去重:两面可能共享多条流形边(平面接圆柱壁的两段弧边、倒角面等),
        // 逐边 push 会让邻居重复出现,污染 CountWallEdgeConvexity 等按邻居计数的逻辑。
        auto& adj1 = adjacency[id1];
        if (std::find(adj1.begin(), adj1.end(), id2) == adj1.end())
            adj1.push_back(id2);
        auto& adj2 = adjacency[id2];
        if (std::find(adj2.begin(), adj2.end(), id1) == adj2.end())
            adj2.push_back(id1);
    }
}
