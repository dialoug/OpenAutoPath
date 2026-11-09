// 独立 STEP 毛坯读取；可安全放入后台任务，不访问 XCAF 文档或查看器状态。
#include "toolpath/core/simulation/StockModelLoader.h"

#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>

namespace oap::toolpath {
namespace {

StockModelLoadResult cancelledResult()
{
    StockModelLoadResult result;
    result.status = StockModelLoadStatus::Cancelled;
    result.message = "毛坯 STEP 加载已取消";
    return result;
}

} // namespace

StockModelLoadResult StockModelLoader::loadStep(
    const std::string& sourcePath,
    const ToolpathGenerationControl* control) const
{
    StockModelLoadResult result;
    if (sourcePath.empty()) {
        result.status = StockModelLoadStatus::EmptyPath;
        result.message = "毛坯 STEP 文件路径为空";
        return result;
    }
    if (control && control->cancelled())
        return cancelledResult();

    try {
        STEPControl_Reader reader;
        if (reader.ReadFile(sourcePath.c_str()) != IFSelect_RetDone) {
            result.status = StockModelLoadStatus::ReadFailed;
            result.message = "无法读取毛坯 STEP 文件";
            return result;
        }
        if (control && control->cancelled())
            return cancelledResult();
        if (reader.TransferRoots() <= 0) {
            result.status = StockModelLoadStatus::TransferFailed;
            result.message = "毛坯 STEP 文件不包含可转换的根形体";
            return result;
        }
        if (control && control->cancelled())
            return cancelledResult();
        result.shape = reader.OneShape();
        if (result.shape.IsNull()) {
            result.status = StockModelLoadStatus::EmptyShape;
            result.message = "毛坯 STEP 转换后得到空形体";
            return result;
        }
        result.status = StockModelLoadStatus::Ok;
        result.message = "毛坯 STEP 加载完成";
        return result;
    } catch (const Standard_Failure&) {
        result.status = StockModelLoadStatus::TransferFailed;
        result.message = "OCCT 在加载毛坯 STEP 时发生异常";
    } catch (...) {
        result.status = StockModelLoadStatus::TransferFailed;
        result.message = "加载毛坯 STEP 时发生未知异常";
    }
    return result;
}

} // namespace oap::toolpath
