// 与策略无关的公共移动表示。几何生成器、预览播放、验证器和导出器
// 通过该类型交换刀路数据。
#pragma once

#include <gp_Pnt.hxx>

namespace oap::toolpath {

enum class ToolpathMoveType {
    Rapid,   // 安全高度上的快速移动
    Plunge,  // 从安全高度进刀到切削深度
    Cut,     // 切削进给
    Retract, // 从切削深度退回安全/清空高度
};

// 在不改变机床移动类型的前提下，记录安全验证所需的生成语义。
// 例如 G83 的孔内重入仍由 G0 执行，但其切削部允许进入已加工孔空间。
enum class ToolpathMoveIntent {
    General,
    DrillCycleReentry,
    PocketLinearRampEntry,
    PocketHelicalRampEntry,
};

// 单条移动：类型、起终点和元数据。生成器按机床无关语义填写；
// loopSequence/depthIndex/ring 仅为元数据，不参与几何计算。
struct ToolpathMove {
    ToolpathMoveType type = ToolpathMoveType::Rapid;
    gp_Pnt from;
    gp_Pnt to;
    int loopSequence = -1; // 生成器分配的稳定操作/环序号。
    int depthIndex = 0;    // 从零开始的深度层元数据，不是机床坐标。
    int ring = 0;          // 偏置环元数据；孔加工操作取零。
    double feed = 0.0;     // 单位 mm/min；快移时为零表示采用机床默认速度。
    ToolpathMoveIntent intent = ToolpathMoveIntent::General;
};

} // namespace oap::toolpath
