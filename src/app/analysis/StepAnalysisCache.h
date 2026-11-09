// STEP 模型快照缓存。
// 缓存目录:<应用主目录>/cache/<contentHash>/,每个源文件一个子目录,内含:
//   geometry.xbf  —— BinXCAF 文档(几何 + 颜色 + 名称),跳过 STEP 解析 + Transfer
//   model.bin     —— oap::Model 二进制(见 ModelIO),跳过拓扑索引
//   meta.txt      —— 校验信息(内容哈希 / 各版本号),任一不符即失效
//
// 命中判定用文件内容 SHA-256(流式读取),避免 mtime 误判和弱哈希碰撞。
// 写缓存失败(目录不可写等)一律静默忽略,绝不影响正常加载。
#pragma once

#include <string>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>

#include "analysis/Model.h"

namespace oap {

// STEP 模型缓存：按内容缓存拓扑数据，避免重复加载。
class StepAnalysisCache {
public:
    // 缓存格式版本。改动缓存布局 / meta 字段时 +1,使旧缓存整体失效。
    static constexpr unsigned kCacheVersion = 2;
    // 单用户缓存的默认总量上限。淘汰必须在后台 worker 中执行。
    static constexpr unsigned long long kDefaultBudgetBytes =
        1024ull * 1024ull * 1024ull;

    // 缓存根目录:<应用主目录>/cache。应用主体可位于主目录的 bin/ 子目录。
    explicit StepAnalysisCache(const std::string& applicationRoot);

    // 对源文件内容做流式 SHA-256。失败返回空串。
    static std::string contentHash(const std::string& sourcePath);

    // 给定源文件,查缓存是否命中(geometry.xbf + model.bin 均存在且
    // meta 校验通过)。无论命中与否，只要源文件可读都会通过 outContentHash
    // 返回内容哈希，供未命中后的 store 复用。
    bool lookup(const std::string& sourcePath,
                std::string& outContentHash) const;

    // 命中后取出:用 application 打开 geometry.xbf 到 outDocument,并读
    // model.bin 到 outModel。
    // 任一步失败返回 false(调用方应回退到完整加载)。
    bool retrieve(
        const std::string& contentHash,
        Handle(TDocStd_Application) application,
        Handle(TDocStd_Document)& outDocument,
        Model& outModel) const;

    // 未命中时写入:把 document 存为 geometry.xbf、model 存为 model.bin、并写
    // meta.txt。文件系统失败返回 false；OCCT 序列化异常由调用它的 worker 统一
    // 捕获并转为用户可读错误。
    bool store(
        const std::string& contentHash,
        Handle(TDocStd_Application) application,
        const Handle(TDocStd_Document)& document,
        const Model& model) const;

    // 按目录总大小上限做 LRU 淘汰(以命中时刷新的子目录 mtime 近似访问时间)。
    void enforceBudget(unsigned long long maxBytes) const;

private:
    std::string m_root; // <应用主目录>/cache

    std::string cacheDirectoryFor(
        const std::string& contentHash) const;
};

} // namespace oap
