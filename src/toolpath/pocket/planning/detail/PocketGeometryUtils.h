// 腔清规划各阶段共用的内部坐标与诊断辅助函数。
#pragma once

#include <string>

#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

namespace oap::toolpath::detail {

// 把一条诊断消息追加到现有 warning 字符串，多条消息以分号分隔。
void appendWarning(std::string& warning, const std::string& message);

// 将局部 XY 刀具中心点沿区域坐标系的 +Z/开口方向，从底面抬高
// depthOffset。
gp_Pnt pointFrom2dAtDepth(const gp_Pnt2d& point,
                          const gp_Ax3& frame,
                          double depthOffset);

} // namespace oap::toolpath::detail
