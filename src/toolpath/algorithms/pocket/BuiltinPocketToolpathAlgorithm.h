// 内置槽/腔清料算法工厂。
//
// 模块归属:oap_toolpath_algorithms(可替换算法实现与注册表)。
// 职责:以工厂函数隔离 PocketClearing(位于 oap_toolpath_pocket)与算法
// 注册表;返回 const 共享对象,可安全注入工作流/异步执行器。
#pragma once

#include <memory>

namespace oap::toolpath {

class IPocketToolpathAlgorithm;

// 创建内置槽腔清料算法实例(以 kBuiltinPocketAlgorithmId 注册)。
// 返回:可共享的只读算法对象,可注入默认注册表供全程序复用。
std::shared_ptr<const IPocketToolpathAlgorithm>
makeBuiltinPocketToolpathAlgorithm();

} // namespace oap::toolpath
