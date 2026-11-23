// MachiningJobDefinition 的版本化 JSON 存储。运行期刀路和 OCCT 形状不进入
// 项目文件；加载后由孔/槽腔 Job 服务按定义重新生成。
// 序列化格式：{format, schema_version, job}，job 内嵌定义 JSON；
// 定义/执行指纹均基于序列化 JSON 计算，任何字段变化都会改变指纹。
#pragma once


#include <string>
#include "manufacturing/job/MachiningJob.h"

namespace oap::manufacturing {

// MachiningJobDefinition 的版本化 .oapjob JSON 存储（oap_manufacturing 持久化层）。
// 提供序列化/反序列化、文件读写、schema 迁移、结构校验以及多类指纹计算；
// 全部为无状态静态方法，不维护任何缓存。
class MachiningJobStorage {
public:
    static std::string serializeDefinition(
        const MachiningJobDefinition& definition);
    static bool deserializeDefinition(
        const std::string& data,
        MachiningJobDefinition& definition,
        std::string* error = nullptr);

    // 保存到文件的同步读写，通过临时文件原子替换。
    static bool save(const std::string& filePath,
                     const MachiningJobDefinition& definition,
                     std::string* error = nullptr);
    static bool load(const std::string& filePath,
                     MachiningJobDefinition& definition,
                     std::string* error = nullptr);

    // 定义指纹：整个 Job 定义的规范化 JSON 哈希，用于判定定义是否变化。
    static std::string definitionFingerprint(
        const MachiningJobDefinition& definition);
    // 执行指纹按领域隔离；孔草稿不参与任一活动结果的执行身份。
    static std::string holeExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static std::string pocketExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static std::string profileExecutionFingerprint(
        const MachiningJobDefinition& definition);
    static std::string finishingExecutionFingerprint(
        const MachiningJobDefinition& definition);
    // Setup 输入指纹：零件指纹 + 装夹几何/轴向/偏置，用于确认失效判定。
    static std::string setupInputFingerprint(
        const PartDefinition& part,
        const SetupDefinition& setup);
    // 源文件内容指纹（分块 SHA-256），用于零件/碰撞模型变更检测。
    static std::string sourceFileFingerprint(const std::string& filePath,
                                         std::string* error = nullptr);
};

} // namespace oap::manufacturing
