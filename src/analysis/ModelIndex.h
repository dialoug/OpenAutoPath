// ModelIndex — 为一个已分析完成的 oap::Model 建立 id->记录指针 的平均 O(1) 查找。
// UI 无关、不依赖 Qt、不参与序列化(故不影响 ModelIO 的缓存格式)。
//
// 设计:指针指向 Model 内 vector 的元素。Model 在分析完成后不再 mutate,
// 在索引使用期间必须存活。每次加载新模型后调用 build() 重建即可。
// 被索引的 Model 若发生增删、重排或 ID 修改，必须重建索引。
#pragma once

#include "Model.h"

#include <string>
#include <unordered_map>

namespace oap {

// 已分析 Model 的 id→记录指针平均 O(1) 查找索引（文件头有生命周期说明）。
class ModelIndex {
public:
    // 从 model 构建全部映射，成功后替换旧索引；异常时保持旧索引不变。
    // 同类记录的重复 ID 抛出 std::invalid_argument，消息包含类别和 ID。
    // 旧索引保留不代表旧模型仍存活：模型生命周期与稳定性由调用方保证。
    void build(const Model& model);

    // 清空所有映射(不持有 Model 时的安全态)。
    void clear();

    // id -> 记录指针;未命中返回 nullptr。
    const Face*        face(const std::string& faceId) const;
    const Edge*        edge(const std::string& edgeId) const;
    const Wire*        wire(const std::string& wireId) const;

private:
    std::unordered_map<std::string, const Face*>        m_faces;
    std::unordered_map<std::string, const Edge*>        m_edges;
    std::unordered_map<std::string, const Wire*>        m_wires;
};

} // namespace oap
