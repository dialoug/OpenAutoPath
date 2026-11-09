// 轮廓加工区域的几何数据模型（oap_toolpath_profile）。
// RootOuterWireProfileSource 是识别结果映射来的中立加工源，ProfileRegion
// 描述装夹坐标系、加工深度、归一化边界与成品/毛坯截面，SkippedProfileRegion
// 记录跳过原因；由 ProfileRegionBuilder 构建，供 ProfileContour 规划与
// ProfileSafetyValidator 校验消费。
#pragma once

#include <string>
#include <vector>

#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>

#include "toolpath/pocket/geometry/OcctAreaAdapter.h"

namespace oap::toolpath {

// 应用层从自动识别结果映射而来的中立加工源。rootWireId 必须是
// rootFaceId 的真实 outer wire；不接受开放边或人工拼接轮廓。
struct RootOuterWireProfileSource {
    std::string unitId;
    std::string axisLabel;
    int normalDirection = 1;
    std::string rootFaceId;
    std::string rootWireId;
    std::vector<std::string> sourceFaceIds;
    // 只用于确定当前装夹方向的加工深度；sourceFaceIds 仍用于逐层截面闭合。
    std::vector<std::string> machiningFaceIds;
    std::vector<std::string> terminalFaceIds;
    std::vector<std::string> mainOuterFaceIds;
};

// 轮廓加工区域：装夹坐标系、加工深度、边界与成品/毛坯截面。
struct ProfileRegion {
    RootOuterWireProfileSource source;
    gp_Dir openDir;
    gp_Ax3 frame; // 原点在最深加工层，+Z 指向工件外侧
    double depth = 0.0;
    Loop2d boundary; // 归一化为 CCW，材料位于轮廓左侧
    // 当前外轮廓特征对应的初始毛坯截面。默认由根外环的局部包围矩形构造；
    // 外部清料只在 stockBoundary 与逐层累计成品包络之间生成刀路。
    Loop2d stockBoundary;
    TopoDS_Shape protectedMaterialShape;
    // 当前 Profile 来源面组的只读副本。逐层走刀边界只能由该面组决定；
    // 完整实体仍单独用于成品材料保护，避免相反装夹面的结构串入。
    TopoDS_Compound sectionShape;
    // 复杂外边界（圆角、降台或开口台阶）不能把根面的 outer wire 等截面
    // 贯穿到最深层；生成器必须在每个深度使用最终实体的真实最外截面。
    bool useMaterialSectionBoundaryAtEveryDepth = false;
    double sectionTolerance = 1.0e-7;
    double sectionDeflection = 0.01;
};

// 跳过的轮廓区域：来源与跳过原因。
struct SkippedProfileRegion {
    RootOuterWireProfileSource source;
    std::string reason;
};

} // namespace oap::toolpath
