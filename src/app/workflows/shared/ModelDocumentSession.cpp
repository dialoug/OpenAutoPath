// ModelDocumentSession 实现（oap_app_workflow_base）。
// 应用持有的已分析 STEP 文档会话，见 ModelDocumentSession.h。
#include "ModelDocumentSession.h"

#include <utility>

#include <XCAFApp_Application.hxx>

namespace oap::app {

ModelDocumentSession::~ModelDocumentSession()
{
    clear();
}

// 替换流程:先释放旧资源,再写入新对象,最后派生索引与坐标系,
// 保证会话内各对象之间的一致性(documents/model/index 三者同生共死)。
void ModelDocumentSession::replace(
    QString sourcePath,
    const Handle(TDocStd_Document)& document,
    Model model,
    StepIndexer indexer)
{
    clear();
    m_sourcePath = std::move(sourcePath);
    m_document = document;
    m_model = std::move(model);
    m_stepIndexer = std::move(indexer);
    m_modelIndex.build(m_model);
    m_moldFrame = MoldCoordinateFrame::fromModel(m_model);
    for (const Unit& unit : m_model.units)
        m_bounds.merge(unit.bbox);
    m_setupWorkFrames = toolpath::MoldSetupWorkFrames::fromMold(
        m_bounds, m_moldFrame);
}

void ModelDocumentSession::clear() noexcept
{
    if (!m_document.IsNull()) {
        try {
            XCAFApp_Application::GetApplication()->Close(m_document);
        } catch (...) {
            // 析构/重置必须保持 noexcept;即使 OCCT 应用报告关闭失败,
            // 下面的代码仍会释放句柄。
        }
    }
    m_document.Nullify();
    m_sourcePath.clear();
    m_model = Model{};
    m_modelIndex.clear();
    m_stepIndexer = StepIndexer{};
    m_moldFrame = MoldCoordinateFrame{};
    m_setupWorkFrames = toolpath::MoldSetupWorkFrames{};
    m_bounds = Bbox{};
}

} // namespace oap::app
