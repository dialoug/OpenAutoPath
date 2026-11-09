// MainWindow 外的毛坯/目标体构建回归测试，仅使用程序化 OCCT 实体。
#include "app/workflows/hole/HoleStockGeometryBuilder.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS.hxx>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::Bbox bounds(double size)
{
    oap::Bbox result;
    result.xmin = 0.0;
    result.ymin = 0.0;
    result.zmin = 0.0;
    result.xmax = size;
    result.ymax = size;
    result.zmax = size;
    return result;
}

bool initialStockRecoversBoundsAndRejectsBadMargins()
{
    const TopoDS_Shape first =
        BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape second = BRepPrimAPI_MakeBox(
        gp_Pnt(20.0, 0.0, 0.0), 5.0, 5.0, 5.0).Shape();
    std::vector<oap::app::HoleStockUnitGeometry> units{
        {"unit-1", bounds(10.0), first},
        {"unit-2", oap::Bbox{}, second},
    };

    const auto aggregate =
        oap::app::HoleStockGeometryBuilder::buildInitialStock(
            units, 1.0, 2.0, 3.0, false);
    if (!expect(aggregate.ok() && aggregate.solidCount == 1,
                "aggregate stock should produce one enclosing solid")) {
        return false;
    }

    const auto perUnit =
        oap::app::HoleStockGeometryBuilder::buildInitialStock(
            units, 0.0, 0.0, 0.0, true);
    if (!expect(perUnit.ok() && perUnit.solidCount == 2 &&
                    !perUnit.warnings.empty(),
                "per-unit stock should recover invalid recorded bounds")) {
        return false;
    }

    const auto invalid =
        oap::app::HoleStockGeometryBuilder::buildInitialStock(
            units,
            std::numeric_limits<double>::quiet_NaN(),
            0.0,
            0.0,
            false);
    return expect(!invalid.ok() && !invalid.error.empty(),
                  "non-finite margins should be rejected before OCCT work");
}

bool targetShapeDeduplicatesAndFallsBack()
{
    const TopoDS_Solid selected = TopoDS::Solid(
        BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());
    const TopoDS_Solid fallback = TopoDS::Solid(
        BRepPrimAPI_MakeBox(
            gp_Pnt(20.0, 0.0, 0.0), 5.0, 5.0, 5.0).Shape());

    const auto direct =
        oap::app::HoleStockGeometryBuilder::buildTargetShape(
            {selected, selected}, {fallback});
    if (!expect(direct.ok() && direct.solidCount == 1,
                "selected target solids should be topology-deduplicated")) {
        return false;
    }

    const auto recovered =
        oap::app::HoleStockGeometryBuilder::buildTargetShape(
            {}, {fallback, fallback});
    if (!expect(recovered.ok() && recovered.solidCount == 1,
                "missing source indices should fall back to model solids")) {
        return false;
    }

    const auto missing =
        oap::app::HoleStockGeometryBuilder::buildTargetShape({}, {});
    return expect(!missing.ok() && !missing.error.empty(),
                  "an empty target should return a diagnostic");
}

} // namespace

int main()
{
    if (!initialStockRecoversBoundsAndRejectsBadMargins() ||
        !targetShapeDeduplicatesAndFallsBack()) {
        return 1;
    }
    std::cout << "stock geometry builder smoke tests passed\n";
    return 0;
}
