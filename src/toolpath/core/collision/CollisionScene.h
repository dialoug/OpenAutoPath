// 固定三轴和后续机床仿真共享的碰撞场景。场景保留每个实体的角色与稳定 ID，
// 使碰撞规则能够区分工件、夹具和机床结构，而不是只面对一个无来源的 Compound。
#pragma once

#include <string>
#include <utility>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace oap::toolpath {

// 碰撞实体角色：区分工件、夹具与机床结构，碰撞规则可按角色采用不同
// 容差或完全忽略某些组合。
enum class CollisionBodyRole {
    Workpiece,
    Fixture,
    Machine,
};

// 碰撞体：角色（工件/夹具）与 TopoDS 形状。
struct CollisionBody {
    std::string id;
    CollisionBodyRole role = CollisionBodyRole::Workpiece;
    TopoDS_Shape shape;
};

// 碰撞场景：按角色与稳定 ID 持有全部实体的值类型容器。
// 由调用方逐次 addBody 构建后整体传给碰撞规划/检查；不持有 OCCT
// 文档或线程资源，可安全跨线程按值传递（内部仅共享句柄）。
class CollisionScene {
public:
    // 是否尚无任何实体。
    bool empty() const noexcept { return m_bodies.empty(); }
    // 全部实体的只读列表，保持添加顺序。
    const std::vector<CollisionBody>& bodies() const noexcept {
        return m_bodies;
    }

    void clear() { m_bodies.clear(); }
    // 追加实体；空形状被忽略。ID 唯一性由调用方保证。
    void addBody(std::string id,
                 CollisionBodyRole role,
                 const TopoDS_Shape& shape)
    {
        if (shape.IsNull())
            return;
        m_bodies.push_back({std::move(id), role, shape});
    }

private:
    std::vector<CollisionBody> m_bodies;
};

} // namespace oap::toolpath
