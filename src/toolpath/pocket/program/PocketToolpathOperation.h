// 槽/腔刀路操作的公共数据结构。生成、验证、预览和导出均依赖该结构，
// 因此它独立于具体导出器定义。
//
// 属于 oap_toolpath_pocket 的程序适配层：由 planning 层生成、被
// simulation/validation/program 各层共同消费；不参与 OCCT 几何内核之外
// 的持久化格式，Job 序列化使用中立 ToolpathProgram。
#pragma once

#include <string>
#include <vector>

#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/core/ToolpathToolContext.h"

namespace oap::toolpath {

// 一条槽/腔工序的完整描述：标识、刀轴、刀具上下文、特征边界与
// 清料结果（loops/moves/primitives）。不变量：clearing.ok 为 true 时
// moves/primitives 与 depthLayerCount 等字段保持一致（由验证器检查）。
struct PocketToolpathOperation {
    std::string operationId;
    std::string algorithmId;
    FeatureKind kind = FeatureKind::Pocket;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    // 自动多刀开粗时冻结到具体工序的刀库刀具。valid=false 保持旧版
    // 手工单刀语义，由调用方使用批次级 ToolpathParams 兜底。
    ToolpathToolContext tool;
    // 0 表示从完整待清区域开始；大于当前刀径时，本工序只加工该上一刀
    // 无法覆盖、但当前刀可达的余料区域。
    double restMachiningPreviousToolDiameter = 0.0;
    // 原始 PocketRegion 下标。多刀开粗会为同一区域产生多道工序，不能再
    // 依赖 operations 与 regions 一一对应。
    int sourceRegionIndex = -1;
    // 从刀尖指向主轴/槽腔开口的固定刀轴，用于材料去除扫掠。
    gp_Vec outwardToolAxis;
    std::vector<std::string> sourceFaceIds;
    std::string floorFaceId;
    // 运行期特征边界面，仅用于建立局部余量审计域；不进入 Job 持久化。
    TopoDS_Shape featureBoundaryShape;
    PocketClearingResult clearing;
};

} // namespace oap::toolpath
