// STEP 模型快照缓存实现。
#include "StepAnalysisCache.h"
#include "analysis/ModelIO.h"
#include "analysis/ContentHash.h"
#include "analysis/Utf8Path.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include <PCDM_StoreStatus.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <TCollection_ExtendedString.hxx>
#include <QElapsedTimer>
#include <QDebug>

namespace fs = std::filesystem;

namespace oap {
namespace {

// 缓存仅包含模型拓扑；ModelIO 版本变化会自动废弃旧分析缓存。
constexpr unsigned kSchemaVersion = 4; // 与 ModelIO 的字节布局版本相互独立。

} // namespace

StepAnalysisCache::StepAnalysisCache(
    const std::string& applicationRoot)
{
    std::error_code ec;
    const fs::path cacheRoot =
        utf8Path(applicationRoot) / "cache";
    // 失败也无妨:后续 store 会再试并静默失败。
    fs::create_directories(cacheRoot, ec);
    m_root = pathUtf8(cacheRoot);
}

std::string StepAnalysisCache::contentHash(
    const std::string& sourcePath)
{
    std::string hash;
    return sha256FileHex(sourcePath, hash) ? hash : std::string{};
}

// contentHash -> 缓存子目录路径(缓存根/<contentHash>)。
std::string StepAnalysisCache::cacheDirectoryFor(
    const std::string& contentHash) const
{
    return pathUtf8(utf8Path(m_root) / contentHash);
}

bool StepAnalysisCache::lookup(
    const std::string& sourcePath,
    std::string& outContentHash) const
{
    outContentHash.clear();
    if (m_root.empty()) return false;
    const std::string sourceHash =
        contentHash(sourcePath);
    if (sourceHash.empty()) return false;
    outContentHash = sourceHash;
    const fs::path cacheDirectory =
        utf8Path(cacheDirectoryFor(sourceHash));
    std::error_code ec;
    if (!fs::exists(cacheDirectory / "geometry.xbf", ec) ||
        !fs::exists(cacheDirectory / "model.bin", ec)) {
        return false;
    }
    // 校验 meta:两行 = 内容哈希 / 版本串。任一不符即失效。
    std::ifstream metadata(
        cacheDirectory / "meta.txt",
        std::ios::binary);
    if (!metadata) return false;
    std::string storedHash;
    std::string storedVersion;
    std::getline(metadata, storedHash);
    std::getline(metadata, storedVersion);
    std::ostringstream expectedVersion;
    expectedVersion
        << kCacheVersion << '.'
        << kSchemaVersion << '.'
        << kModelIOFormatVersion;
    return storedHash == sourceHash &&
           storedVersion == expectedVersion.str();
}

bool StepAnalysisCache::retrieve(
    const std::string& contentHash,
    Handle(TDocStd_Application) application,
    Handle(TDocStd_Document)& outDocument,
    Model& outModel) const
{
    if (m_root.empty() || application.IsNull())
        return false;
    const fs::path cacheDirectory =
        utf8Path(cacheDirectoryFor(contentHash));
    // 读几何:BinXCAF 文档。
    QElapsedTimer timer;
    timer.start();
    const std::string documentPathUtf8 =
        pathUtf8(cacheDirectory / "geometry.xbf");
    const TCollection_ExtendedString documentPath(
        documentPathUtf8.c_str(), Standard_True);
    const PCDM_ReaderStatus readerStatus =
        application->Open(documentPath, outDocument);
    if (readerStatus != PCDM_RS_OK ||
        outDocument.IsNull()) {
        return false;
    }
    const qint64 documentOpenMillis =
        timer.restart();
    // 读拓扑 Model。
    if (!readModel(
            pathUtf8(cacheDirectory / "model.bin"),
            outModel)) {
        return false;
    }
    const qint64 modelReadMillis =
        timer.elapsed();
    // lookup 只验证元数据，实际成功读取后才算一次访问。刷新目录 mtime 使
    // enforceBudget 的淘汰顺序与真实使用顺序一致。
    std::error_code ec;
    fs::last_write_time(
        cacheDirectory,
        fs::file_time_type::clock::now(), ec);
    qDebug().nospace()
        << "[OpenAutoPath] StepAnalysisCache::retrieve ms: xbf_open="
        << documentOpenMillis
        << " model_bin_read=" << modelReadMillis;
    return true;
}

bool StepAnalysisCache::store(
    const std::string& contentHash,
    Handle(TDocStd_Application) application,
    const Handle(TDocStd_Document)& document,
    const Model& model) const
{
    if (m_root.empty() || application.IsNull() ||
        document.IsNull() || contentHash.empty()) {
        return false;
    }
    std::error_code ec;
    const fs::path cacheDirectory =
        utf8Path(cacheDirectoryFor(contentHash));
    const fs::path stagingDirectory =
        utf8Path(m_root) / (contentHash + ".tmp");
    const fs::path backupDirectory =
        utf8Path(m_root) / (contentHash + ".old");
    fs::remove_all(stagingDirectory, ec);
    ec.clear();
    if (!fs::create_directories(stagingDirectory, ec))
        return false;

    // 几何:存为 BinXCAF。需先把文档存储格式切到 "BinXCAF"(由 DefineFormat 注册)。
    Handle(TDocStd_Document) documentToWrite =
        document;
    documentToWrite->ChangeStorageFormat(
        TCollection_ExtendedString("BinXCAF"));
    const std::string documentPathUtf8 =
        pathUtf8(stagingDirectory / "geometry.xbf");
    const TCollection_ExtendedString documentPath(
        documentPathUtf8.c_str(), Standard_True);
    const PCDM_StoreStatus storeStatus =
        application->SaveAs(
            documentToWrite, documentPath);
    if (storeStatus != PCDM_SS_OK) {
        fs::remove_all(stagingDirectory, ec);
        return false;
    }

    // 拓扑 Model。
    if (!writeModel(
            pathUtf8(stagingDirectory / "model.bin"),
            model)) {
        fs::remove_all(stagingDirectory, ec);
        return false;
    }

    // meta:两行 = 内容哈希 / 版本串。
    {
        std::ofstream metadata(
            stagingDirectory / "meta.txt",
            std::ios::binary | std::ios::trunc);
        if (!metadata) {
            fs::remove_all(stagingDirectory, ec);
            return false;
        }
        std::ostringstream version;
        version
            << kCacheVersion << '.'
            << kSchemaVersion << '.'
            << kModelIOFormatVersion;
        metadata
            << contentHash << '\n'
            << version.str() << '\n';
        if (!metadata) {
            fs::remove_all(stagingDirectory, ec);
            return false;
        }
    }

    // 先把旧快照改名为备份，再提交完整的新目录。新目录提交失败时恢复旧快照，
    // 避免“删除旧缓存后进程退出”导致原本有效的快照丢失。
    fs::remove_all(backupDirectory, ec);
    ec.clear();
    const bool hadPrevious = fs::exists(cacheDirectory, ec);
    if (ec) {
        std::error_code cleanupError;
        fs::remove_all(stagingDirectory, cleanupError);
        return false;
    }
    if (hadPrevious) {
        fs::rename(
            cacheDirectory, backupDirectory, ec);
        if (ec) {
            fs::remove_all(stagingDirectory, ec);
            return false;
        }
    }
    fs::rename(stagingDirectory, cacheDirectory, ec);
    if (ec) {
        if (hadPrevious) {
            std::error_code restoreError;
            fs::rename(backupDirectory, cacheDirectory, restoreError);
        }
        std::error_code cleanupError;
        fs::remove_all(stagingDirectory, cleanupError);
        return false;
    }
    if (hadPrevious) {
        std::error_code cleanupError;
        fs::remove_all(backupDirectory, cleanupError);
    }
    return true;
}

void StepAnalysisCache::enforceBudget(
    unsigned long long maxBytes) const
{
    if (m_root.empty()) return;
    // 收集各缓存子目录的(总大小, mtime),按 mtime 升序淘汰直到总量 <= 上限。
    struct CacheEntry {
        fs::path directory;
        unsigned long long bytes = 0;
        fs::file_time_type lastAccessTime;
    };
    std::vector<CacheEntry> entries;
    unsigned long long totalBytes = 0;
    std::error_code iterationError;
    for (fs::directory_iterator directoryIt(
             m_root, iterationError), end;
         directoryIt != end;
         directoryIt.increment(iterationError)) {
        if (iterationError) break;

        std::error_code typeError;
        if (!directoryIt->is_directory(typeError) ||
            typeError) {
            continue;
        }

        unsigned long long entryBytes = 0;
        std::error_code recursiveError;
        for (fs::recursive_directory_iterator fileIt(
                 directoryIt->path(), recursiveError), fileEnd;
             fileIt != fileEnd;
             fileIt.increment(recursiveError)) {
            if (recursiveError) break;

            std::error_code fileError;
            if (fileIt->is_regular_file(fileError) &&
                !fileError) {
                const std::uintmax_t fileBytes =
                    fileIt->file_size(fileError);
                if (!fileError) {
                    entryBytes +=
                        static_cast<unsigned long long>(
                            fileBytes);
                }
            }
        }

        std::error_code timeError;
        const fs::file_time_type lastAccessTime =
            fs::last_write_time(
                directoryIt->path(), timeError);
        if (timeError)
            continue;
        entries.push_back(
            {directoryIt->path(), entryBytes,
             lastAccessTime});
        totalBytes += entryBytes;
    }
    if (totalBytes <= maxBytes) return;

    std::sort(
        entries.begin(), entries.end(),
        [](const CacheEntry& first,
           const CacheEntry& second) {
            return first.lastAccessTime <
                   second.lastAccessTime;
        });
    for (const CacheEntry& entry : entries) {
        if (totalBytes <= maxBytes) break;
        std::error_code removalError;
        fs::remove_all(
            entry.directory, removalError);
        if (!removalError) {
            totalBytes -=
                std::min(totalBytes, entry.bytes);
        }
    }
}

} // namespace oap
