// FeatureRecognizer — 把 AAG 孔识别算法桥接到 oap 分析模型。
// 用各 unit 的 solid 组装成 compound 跑 AAG 识别,再把 AAG 的 1-based 面索引
// 经 StepIndexer::idForFace() 反查回 "F<n>",得到 face_id -> 特征类别 的映射。
// 模块: oap_recognition（依赖 oap_analysis）。本类是对 app 层的识别门面，
//       消费方为 oap_app_recognition / oap_app_workflow_base / oap_app_workflow_cam。
#pragma once

#include <map>
#include <string>
#include <vector>

#include <Quantity_Color.hxx>

#include "analysis/FeatureTypes.h"
#include "analysis/HoleGeometry.h"
#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

namespace oap {

// 特征类别 -> 识别结果视图颜色
// (通孔=绿、盲孔=蓝、沉孔=黄;槽=红、腔=青)。
Quantity_Color featureColor(FeatureKind kind);

// 一次完整识别的事务结果：instances 为权威数据，faceKind / 各类别计数由
// rebuildDerived() 派生；任何修改 instances 的操作后都应重新派生。
struct RecognitionResult {       
    // 保留原有嵌套类型名，现有 UI/识别代码无需感知公共类型的迁移。
    using Instance = FeatureInstance;

    // 权威数据:全部特征实例,按类别枚举序、再按首面排序。
    std::vector<Instance> instances;

    // 与本次完整识别共用同一份 AAG 和 HoleResult 生成的纯孔几何。
    std::vector<RecognizedHoleGeometry> holeGeometries;

    // face_id -> 该面所属特征类别(从 instances 派生;孔优先,一个面只归一类)。
    // 保留以兼容染色 / 拾取等按面查类别的现有逻辑。
    std::map<std::string, FeatureKind> faceKind;
    int holeCount = 0;
    int slotCount = 0;
    int pocketCount = 0;
    // 从 instances 重建 faceKind 与各类别实例计数。任何修改 instances 后调用。
    void rebuildDerived();

    // 去重:移除面集完全相同的重复实例,以及被另一实例真包含的子集实例。
    // 此步也用于整理人工复核产生的重复或包含实例。
    void dedupInstances();
};

// 仅孔几何的识别结果，附带 AAG 构建、孔分类、几何构建的分段耗时（ms），
// 供界面进度与统计展示使用。
struct HoleGeometryRecognitionResult {
    std::vector<RecognizedHoleGeometry> geometries;
    long long aagBuildMs = 0;
    long long holeClassificationMs = 0;
    long long geometryBuildMs = 0;
};

// 特征识别器：对已索引模型运行孔识别，并支持按选中面做孔几何识别。
class FeatureRecognizer {
public:
    // Optional process-wide presentation sink. Install before starting workers;
    // reset after workers stop. The core keeps a stderr fallback and no Qt dependency.
    using DiagnosticHandler = void (*)(const char* prefix, const char* detail);
    static void setDiagnosticHandler(DiagnosticHandler handler) noexcept;
    // 对已索引的 model 运行孔识别。槽、腔等类别只保留人工标记与下游加工支持。
    RecognitionResult recognize(const Model& model, const StepIndexer& indexer);

    // 仅识别孔加工所需几何。返回顺序与完整 recognize() 中的孔顺序一致。
    HoleGeometryRecognitionResult recognizeHoleGeometries(
        const Model& model, const StepIndexer& indexer);

    std::vector<RecognizedHoleGeometry> recognizeSelectedHoleGeometries(
        const Model& model,
        const StepIndexer& indexer,
        const std::vector<std::string>& faceIds,
        std::string* error = nullptr);

};

} // namespace oap
