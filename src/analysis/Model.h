// 分析数据模型 — 与 docs/07_output_contract_phase1.md 对应、独立于 UI 的记录。
// ID 采用契约中的 "U<n>" / "F<n>" / "E<n>" / "W<n>" 方案。
#pragma once

#include <array>
#include <string>
#include <vector>

namespace oap {

// 世界坐标系下的轴对齐包围盒:[xmin,ymin,zmin,xmax,ymax,zmax]。
struct Bbox {
    double xmin = 0, ymin = 0, zmin = 0;
    double xmax = -1, ymax = -1, zmax = -1; // 默认为"无效"(max < min)

    bool valid() const noexcept { return xmax >= xmin && ymax >= ymin && zmax >= zmin; }
    std::array<double, 3> center() const noexcept {
        return {(xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2};
    }
    std::array<double, 3> size() const noexcept {
        return {xmax - xmin, ymax - ymin, zmax - zmin};
    }
    // 扩展此包围盒使其也包含另一个(用于从面派生出 unit_bbox)。
    void merge(const Bbox& o) noexcept {
        if (!o.valid()) return;
        if (!valid()) { *this = o; return; }
        xmin = std::min(xmin, o.xmin); ymin = std::min(ymin, o.ymin); zmin = std::min(zmin, o.zmin);
        xmax = std::max(xmax, o.xmax); ymax = std::max(ymax, o.ymax); zmax = std::max(zmax, o.zmax);
    }
};

// 一个实体/体。名称字段遵循 doc 08;角色提示仅为参考(绝不覆盖几何判定)。
struct Unit {
    std::string unit_id;          // "U<n>"
    std::string unit_name;        // 来自 STEP 的原始名称
    std::string decoded_name;     // 经 \X2\..\X0\ 解码 + 去空白后
    std::string name_prefix;      // 例如 "UP"、"MI_LP"
    std::string name_family;      // 例如 "upper_plate_family"
    std::string unit_kind;        // 单元类型：solid / shell / fallback_component / unknown
    std::string unit_geometry_class; // 单元几何分类：plate_like_unit / block_like_unit / ...
    std::string unit_role_hint;   // 单元角色提示：plate_layer_candidate / insert_candidate / ...
    std::string role_hint_source; // 角色提示来源：name_rule / geometry_rule / fallback / conflict
    double role_hint_confidence = 0.0;
    std::vector<std::string> role_hint_diagnostics;
    Bbox bbox;
    std::vector<std::string> face_ids;
};

// 一个 TopoDS_Face。仅含几何事实 — 无特征/标签/置信度(doc 07 §7 在此禁止这些)。
struct Face {
    std::string face_id;          // "F<n>"
    std::string unit_id;
    std::string surface_type;     // 曲面类型：plane / cylinder / cone / sphere / torus / bspline / other
    double area = -1;             // <0 表示未知
    Bbox bbox;
    bool has_center = false;
    std::array<double, 3> center{0, 0, 0};
    bool has_normal = false;
    std::array<double, 3> normal{0, 0, 0};
    double radius = -1;           // <0 表示不适用
    std::vector<std::string> edge_ids;
    std::vector<std::string> wire_ids;
};

// 唯一 edge 记录(共享边在所有 solid 中去重为一条,doc 07 §5)。
struct Edge {
    std::string edge_id;          // "E<n>"
    std::string unit_id;          // 首次遇到该边的 unit
    std::string curve_type;       // 曲线类型：line / circle / ellipse / bspline / other
    double length = -1;           // <0 表示未知
    Bbox bbox;
    std::vector<std::string> face_ids; // 该边参与构成的 face
    std::vector<std::string> wire_ids; // 包含该边的 wire
    bool has_start = false;
    std::array<double, 3> start_point{0, 0, 0};
    bool has_end = false;
    std::array<double, 3> end_point{0, 0, 0};
};

// 一个 face 下的 wire / loop(doc 07 §6)。wire 与 face 绑定,不跨 face 去重。
struct Wire {
    std::string wire_id;          // "W<n>"
    std::string unit_id;
    std::string face_id;
    std::string wire_role;        // 线框角色：outer / inner / unknown
    bool is_closed = false;
    std::vector<std::string> edge_ids; // 有序 edge 引用
    Bbox bbox;
};

// 单个已加载模型的完整分析结果。各 vector 按其 key 稳定排序。
// 只含分析结论(几何与拓扑事实),不含 TopoDS 形状:
// 由 StepIndexer 填充、ModelIO 读写缓存、ModelIndex 建立查找索引。
struct Model {
    std::vector<Unit> units;
    std::vector<Face> faces;
    std::vector<Edge> edges;
    std::vector<Wire> wires;
};

} // namespace oap

