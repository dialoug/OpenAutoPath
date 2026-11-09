// 面邻接属性图公共模型；图节点索引与构造时的面映射保持一致。
// 模块: oap_recognition（依赖 oap_analysis），特征识别共享的基础数据结构。
// 协作: 由孔识别、轴向平面筛选、三轴内外环追踪等识别器以 const 引用复用；
//       实例由识别事务持有，每次 build() 清空并重建全部数据。
// 消费方: FeatureRecognizer / ThreeAxisWireTraceUseCase，以及 Recognition.h
//       中各 OCCT 识别入口。
#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

class TopoDS_Shape;

// 面邻接属性图（AAG）：顶点为 B-Rep 面（1-based，与 faceMap 一致），
// 邻接只登记被恰好两个面共享的流形边。本结构只维护拓扑邻接与共享边缓存，
// 不含凹凸性；凹凸性由各识别器按需经 DihedralAngleChecker 计算并自行缓存。
class AAG
{
public:
    // 所有面（索引访问）
    TopTools_IndexedMapOfShape faceMap;

    // 邻接关系：face index -> 邻接 face index
    std::unordered_map<int, std::vector<int>> adjacency;

public:
    // 重建邻接图：清空现有数据后，基于 shape 遍历 Face/Edge 建立 faceMap、
    // adjacency 和 m_sharedEdges。同一实例可安全地重复调用。
    void build(const TopoDS_Shape& shape);

    // 返回构图时缓存的非退化共享边。键与面顺序无关；同一对面共享多条边时
    // 保留拓扑遍历遇到的第一条有效边，与孔识别原有的首条共享边语义一致。
    const TopoDS_Edge* sharedEdge(
        int firstFaceIndex,
        int secondFaceIndex) const;

private:
    // 无序面下标对 -> 64 位键（较小者放高 32 位），保证双向查询一致。
    static std::uint64_t facePairKey(
        int firstFaceIndex,
        int secondFaceIndex);

    std::unordered_map<std::uint64_t, TopoDS_Edge>
        m_sharedEdges;
};
