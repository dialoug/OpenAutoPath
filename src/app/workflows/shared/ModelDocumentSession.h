// 应用持有的已分析 STEP 文档会话。
//
// 会话拥有相互依赖的 document/model/index 对象,并保证每当模型被替换时
// 都会重建 ModelIndex。它不包含任何 QWidget、场景或展示状态。
#pragma once

#include <QString>

#include <TDocStd_Document.hxx>

#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/StepIndexer.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::app {

// 模型文档会话:由应用顶层持有,独占整个 STEP 文档生命周期,
// 所有读/写访问器直接暴露内部引用,替换文档必须走 replace()。
class ModelDocumentSession {
public:
    ModelDocumentSession() = default;
    ~ModelDocumentSession();

    ModelDocumentSession(const ModelDocumentSession&) = delete;
    ModelDocumentSession& operator=(const ModelDocumentSession&) = delete;

    // 原子替换文档:先清空旧文档,再装入新模型/索引器,
    // 重建 ModelIndex、模具坐标系与工位坐标系(随包围盒推导)。
    void replace(
        QString sourcePath,
        const Handle(TDocStd_Document)& document,
        Model model,
        StepIndexer indexer);
    // 清空全部内容并关闭 OCCT 文档(保证 noexcept)。
    void clear() noexcept;

    bool empty() const noexcept { return m_document.IsNull(); }
    const QString& sourcePath() const noexcept { return m_sourcePath; }
    QString& sourcePath() noexcept { return m_sourcePath; }

    const Handle(TDocStd_Document)& document() const noexcept
    {
        return m_document;
    }
    Handle(TDocStd_Document)& document() noexcept { return m_document; }

    const Model& model() const noexcept { return m_model; }
    Model& model() noexcept { return m_model; }
    const ModelIndex& modelIndex() const noexcept { return m_modelIndex; }
    ModelIndex& modelIndex() noexcept { return m_modelIndex; }
    const StepIndexer& stepIndexer() const noexcept { return m_stepIndexer; }
    StepIndexer& stepIndexer() noexcept { return m_stepIndexer; }
    const MoldCoordinateFrame& moldFrame() const noexcept
    {
        return m_moldFrame;
    }
    MoldCoordinateFrame& moldFrame() noexcept { return m_moldFrame; }
    const toolpath::MoldSetupWorkFrames& setupWorkFrames() const noexcept
    {
        return m_setupWorkFrames;
    }
    toolpath::MoldSetupWorkFrames& setupWorkFrames() noexcept
    {
        return m_setupWorkFrames;
    }
    const Bbox& bounds() const noexcept { return m_bounds; }

private:
    QString m_sourcePath;
    Handle(TDocStd_Document) m_document;
    Model m_model;
    ModelIndex m_modelIndex;
    StepIndexer m_stepIndexer;
    MoldCoordinateFrame m_moldFrame;
    toolpath::MoldSetupWorkFrames m_setupWorkFrames;
    Bbox m_bounds;
};

} // namespace oap::app
