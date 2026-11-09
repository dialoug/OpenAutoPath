#include "StepIndexGeometry.h"

#include <chrono>
#include <mutex>
#include <vector>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <Bnd_Box.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace oap::detail {
namespace {
using Clock = std::chrono::steady_clock;
std::int64_t millis(Clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start).count();
}
Bbox exactBounds(const TopoDS_Shape& shape) {
    Bbox result;
    try {
        Bnd_Box box;
        BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False);
        if (!box.IsVoid())
            box.Get(result.xmin,result.ymin,result.zmin,result.xmax,result.ymax,result.zmax);
    } catch (...) { return {}; }
    return result;
}
bool complexFace(const TopoDS_Shape& face) {
    try {
        const auto type = BRepAdaptor_Surface(TopoDS::Face(face)).GetType();
        return type != GeomAbs_Plane && type != GeomAbs_Cylinder &&
               type != GeomAbs_Cone && type != GeomAbs_Sphere && type != GeomAbs_Torus;
    } catch (...) { return false; }
}
}

bool buildFaceBounds(const Handle(TDocStd_Document)& document,
                     const StepIndexBuildControl& control,
                     StepIndexBuildStatistics& statistics,
                     FaceBoundsMap& bounds) {
    statistics = {};
    const auto preparationStart = Clock::now();
    auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    TDF_LabelSequence labels;
    shapeTool->GetShapes(labels);
    TopTools_IndexedMapOfShape faces;
    // 与索引相同，只处理 solid/shell；按 TShape+Location 去重。
    for (const auto& label : labels) {
        if (control.cancelled()) return false;
        const auto shape = shapeTool->GetShape(label);
        if (shape.IsNull()) continue;
        for (auto kind : {TopAbs_SOLID, TopAbs_SHELL})
            for (TopExp_Explorer units(shape,kind); units.More(); units.Next())
                for (TopExp_Explorer face(units.Current(),TopAbs_FACE); face.More(); face.Next())
                    faces.Add(face.Current());
    }
    const auto count = static_cast<std::size_t>(faces.Extent());
    statistics.faceCount = count;
    std::vector<Bbox> results(count);
    std::vector<TopoDS_Shape> isolated(count);
    std::vector<std::size_t> parallel;
    if (control.executeRange) {
        for (std::size_t i=0; i<count; ++i)
            if (complexFace(faces(static_cast<int>(i+1)))) parallel.push_back(i);
    }
    // 小任务留在当前线程；几何深拷贝完成后才允许并发，避免共享曲面内部状态。
    if (parallel.size() < 4) parallel.clear();
    for (auto i : parallel) {
        if (control.cancelled()) return false;
        try {
            isolated[i] = BRepBuilderAPI_Copy(faces(static_cast<int>(i+1)), true, false).Shape();
        } catch (...) {
            // 无法隔离的面由下面的串行路径计算，沿用旧版坏面容错。
        }
        if (!isolated[i].IsNull()) ++statistics.isolatedFaceCount;
    }
    statistics.boundsPreparationMillis = millis(preparationStart);
    const auto boundsStart = Clock::now();
    std::mutex progressMutex;
    std::size_t completed = 0;
    const auto report = [&] {
        std::lock_guard lock(progressMutex);
        ++completed;
        if (control.reportProgress) control.reportProgress(completed,count);
    };
    if (control.reportProgress) control.reportProgress(0,count);
    for (std::size_t i=0; i<count; ++i) {
        if (control.cancelled()) return false;
        if (isolated[i].IsNull()) {
            results[i] = exactBounds(faces(static_cast<int>(i+1)));
            report();
        }
    }
    if (!parallel.empty()) control.executeRange(parallel.size(), [&](std::size_t task) {
        const auto i = parallel[task];
        if (control.cancelled() || isolated[i].IsNull()) return;
        results[i] = exactBounds(isolated[i]);
        report();
    });
    statistics.faceBoundsMillis = millis(boundsStart);
    if (control.cancelled()) return false;
    // OCCT 映射只在此串行写入，工作线程只访问自己的结果槽。
    for (std::size_t i=0; i<count; ++i)
        bounds.Bind(faces(static_cast<int>(i+1)),results[i]);
    return true;
}
}
