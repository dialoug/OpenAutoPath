// 内置轮廓刀路算法工厂（oap_toolpath_algorithms）。
// makeBuiltinProfileToolpathAlgorithm() 返回注册表默认使用的轮廓算法实现，
// 内部委托 ProfileContour（toolpath/profile/planning）完成刀路生成；
// 实现见 BuiltinProfileToolpathAlgorithm.cpp。
#pragma once

#include <memory>

namespace oap::toolpath {
class IProfileToolpathAlgorithm;

std::shared_ptr<const IProfileToolpathAlgorithm>
makeBuiltinProfileToolpathAlgorithm();
} // namespace oap::toolpath
