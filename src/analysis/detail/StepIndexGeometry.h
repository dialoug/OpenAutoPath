// 单次索引的精确面包围盒预计算；结果仅在同步任务全部结束后发布。
#pragma once
#include "analysis/StepIndexer.h"

namespace oap::detail {
using FaceBoundsMap = NCollection_DataMap<TopoDS_Shape, Bbox, TopTools_ShapeMapHasher>;
bool buildFaceBounds(const Handle(TDocStd_Document)& document,
                     const StepIndexBuildControl& control,
                     StepIndexBuildStatistics& statistics,
                     FaceBoundsMap& bounds);
}
