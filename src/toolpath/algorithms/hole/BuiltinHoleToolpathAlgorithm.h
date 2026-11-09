// 内置孔加工算法工厂。注册表只依赖工厂，不直接构造 HoleMachining。
//
// 模块归属:oap_toolpath_algorithms(可替换算法实现与注册表)。
// 职责:以工厂函数隔离 HoleMachining(位于 oap_toolpath_hole)与算法
// 注册表,使注册表不必编译期依赖具体生成器。工厂返回 const 共享对象。
#pragma once

#include <memory>

namespace oap::toolpath {

class IHoleToolpathAlgorithm;

// 创建内置孔加工算法实例(以 kBuiltinHoleAlgorithmId 注册)。
// 返回:可共享的只读算法对象,可注入默认注册表供全程序复用。
std::shared_ptr<const IHoleToolpathAlgorithm>
makeBuiltinHoleToolpathAlgorithm();

} // namespace oap::toolpath
