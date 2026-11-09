// 兼容聚合头。新代码应按工序包含 HoleProgramAdapter 或
// PocketProgramAdapter，避免同时依赖两种具体工序。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 本头仅供旧调用方一次性引入两种工序适配器,不定义任何新符号。
#pragma once

#include "toolpath/hole/HoleProgramAdapter.h"
#include "toolpath/pocket/program/PocketProgramAdapter.h"
#include "toolpath/profile/program/ProfileProgramAdapter.h"
