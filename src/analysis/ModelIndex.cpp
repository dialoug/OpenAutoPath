// ModelIndex 实现 — 参见 ModelIndex.h。
#include "ModelIndex.h"

#include <stdexcept>

namespace oap {
namespace {

template <typename Record>
void insertUnique(std::unordered_map<std::string, const Record*>& index,
                  const std::string& id, const Record& record,
                  const char* kind) {
    if (!index.emplace(id, &record).second)
        throw std::invalid_argument(
            std::string("ModelIndex: duplicate ") + kind + " ID: " + id);
}

} // namespace

void ModelIndex::clear() {
    m_faces.clear();
    m_edges.clear();
    m_wires.clear();
}

// 重建全部 id->记录映射。映射中保存指向 model 内元素的裸指针,
// 调用方须保证 model 生命周期覆盖本索引,且其后不再增删 vector 元素。
void ModelIndex::build(const Model& model) {
    // 先构建候选映射；重复 ID 或分配失败时不发布半份索引。
    ModelIndex candidate;
    // reserve 避免大模型重建时反复 rehash。
    candidate.m_faces.reserve(model.faces.size());
    candidate.m_edges.reserve(model.edges.size());
    candidate.m_wires.reserve(model.wires.size());

    for (const auto& f : model.faces)
        insertUnique(candidate.m_faces, f.face_id, f, "face");
    for (const auto& e : model.edges)
        insertUnique(candidate.m_edges, e.edge_id, e, "edge");
    for (const auto& w : model.wires)
        insertUnique(candidate.m_wires, w.wire_id, w, "wire");

    // 这些映射使用标准 hash/equal/allocator，swap 不抛异常。
    m_faces.swap(candidate.m_faces);
    m_edges.swap(candidate.m_edges);
    m_wires.swap(candidate.m_wires);
}

const Face* ModelIndex::face(const std::string& faceId) const {
    auto it = m_faces.find(faceId);
    return it == m_faces.end() ? nullptr : it->second;
}

const Edge* ModelIndex::edge(const std::string& edgeId) const {
    auto it = m_edges.find(edgeId);
    return it == m_edges.end() ? nullptr : it->second;
}

const Wire* ModelIndex::wire(const std::string& wireId) const {
    auto it = m_wires.find(wireId);
    return it == m_wires.end() ? nullptr : it->second;
}

} // namespace oap
