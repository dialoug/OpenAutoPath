// MachiningJobDefinition 的版本化 JSON 存储。运行期刀路和 OCCT 形状不进入
// 项目文件；加载后由孔/槽腔 Job 服务按定义重新生成。
// 序列化格式：{format, schema_version, job}，job 内嵌定义 JSON；
// 定义/执行指纹均基于序列化 JSON 计算，任何字段变化都会改变指纹。
#pragma once

#include <QByteArray>
#include <QString>

#include "manufacturing/job/MachiningJob.h"

namespace oap::manufacturing {

// MachiningJobDefinition 的版本化 .oapjob JSON 存储（oap_manufacturing 持久化层）。
// 提供序列化/反序列化、文件读写、schema 迁移、结构校验以及多类指纹计算；
// 全部为无状态静态方法，不维护任何缓存。
class MachiningJobStorage {
public:
    static QByteArray serializeDefinition(
        const MachiningJobDefinition& definition);
    static bool deserializeDefinition(
        const QByteArray& data,
        MachiningJobDefinition& definition,
        QString* error = nullptr);

    // 保存到文件的同步读写，使用 QSaveFile 保证原子替换。
    static bool save(const QString& filePath,
                     const MachiningJobDefinition& definition,
                     QString* error = nullptr);
    static bool load(const QString& filePath,
                     MachiningJobDefinition& definition,
                     QString* error = nullptr);

    // 定义指纹：整个 Job 定义的规范化 JSON 哈希，用于判定定义是否变化。
    static QString definitionFingerprint(
        const MachiningJobDefinition& definition);
    // 执行指纹按领域隔离；孔草稿不参与任一活动结果的执行身份。
    static QString holeExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static QString pocketExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static QString profileExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static QString finishingExecutionFingerprint(
        const MachiningJobDefinition& definition);
    // Setup 输入指纹：零件指纹 + 装夹几何/轴向/偏置，用于确认失效判定。
    static QString setupInputFingerprint(
        const PartDefinition& part,
        const SetupDefinition& setup);
    // 源文件内容指纹（分块 SHA-256），用于零件/碰撞模型变更检测。
    static QString sourceFileFingerprint(const QString& filePath,
                                         QString* error = nullptr);
};

} // namespace oap::manufacturing
