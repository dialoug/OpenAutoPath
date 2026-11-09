// OverlayController 实现（oap_app_presentation）。
// AIS overlay 的语义分组与唯一注册表，见 OverlayController.h。
#include "app/presentation/OverlayController.h"

#include <stdexcept>
#include <utility>

namespace oap::app::presentation {

OcctPresentationSink::OcctPresentationSink(
    Handle(AIS_InteractiveContext) context)
    : m_context(std::move(context))
{
}

void OcctPresentationSink::bindContext(
    const Handle(AIS_InteractiveContext)& context)
{
    m_context = context;
}

void OcctPresentationSink::display(
    const Handle(AIS_InteractiveObject)& object)
{
    if (!m_context.IsNull() && !object.IsNull())
        m_context->Display(object, Standard_False);
}

void OcctPresentationSink::erase(
    const Handle(AIS_InteractiveObject)& object)
{
    if (!m_context.IsNull() && !object.IsNull())
        m_context->Erase(object, Standard_False);
}

void OcctPresentationSink::remove(
    const Handle(AIS_InteractiveObject)& object)
{
    if (!m_context.IsNull() && !object.IsNull())
        m_context->Remove(object, Standard_False);
}

void OcctPresentationSink::redisplay(
    const Handle(AIS_InteractiveObject)& object)
{
    if (!m_context.IsNull() && !object.IsNull())
        m_context->Redisplay(object, Standard_False);
}

void OcctPresentationSink::deactivate(
    const Handle(AIS_InteractiveObject)& object)
{
    if (!m_context.IsNull() && !object.IsNull())
        m_context->Deactivate(object);
}

void OcctPresentationSink::setDisplayPriority(
    const Handle(AIS_InteractiveObject)& object, int priority)
{
    if (!m_context.IsNull() && !object.IsNull() && priority >= 0)
        m_context->SetDisplayPriority(
            object, static_cast<Graphic3d_DisplayPriority>(priority));
}

bool OcctPresentationSink::isDisplayed(
    const Handle(AIS_InteractiveObject)& object) const
{
    return !m_context.IsNull() && !object.IsNull() &&
        m_context->IsDisplayed(object);
}

OverlayController::OverlayController(
    std::shared_ptr<PresentationSink> sink)
    : m_sink(std::move(sink)),
      m_ownerThread(std::this_thread::get_id())
{
    if (!m_sink)
        throw std::invalid_argument("OverlayController requires a presentation sink");
}

OverlayController::~OverlayController()
{
    // 正常所有者在 UI 线程析构。若外部错误地跨线程销毁，不调用 OCCT context，
    // 避免把一个所有权错误升级为 viewer 的未定义行为。
    if (std::this_thread::get_id() == m_ownerThread)
        clearAllTransient();
}

void OverlayController::requireOwnerThread() const
{
    if (std::this_thread::get_id() != m_ownerThread) {
        throw std::logic_error(
            "OverlayController may only access the presentation sink on its owner thread");
    }
}

// 统一的对象发布步骤:Display 后按选项停用拾取并设置显示优先级。
// 与覆盖组是否可见无关,仅负责“把对象按选项送进 viewer”。
void OverlayController::displayObject(
    const Handle(AIS_InteractiveObject)& object,
    const OverlayDisplayOptions& options)
{
    if (object.IsNull())
        return;
    m_sink->display(object);
    if (options.deactivateSelection)
        m_sink->deactivate(object);
    if (options.displayPriority >= 0)
        m_sink->setDisplayPriority(object, options.displayPriority);
}

void OverlayController::bindContext(
    const Handle(AIS_InteractiveContext)& context)
{
    requireOwnerThread();
    m_sink->bindContext(context);
    for (const auto& [group, state] : m_groups) {
        (void)group;
        if (!state.visible)
            continue;
        for (const Handle(AIS_InteractiveObject)& object : state.objects)
            displayObject(object, state.options);
    }
}

void OverlayController::replace(
    OverlayGroup group,
    std::vector<Handle(AIS_InteractiveObject)> objects,
    bool visible,
    OverlayDisplayOptions options)
{
    requireOwnerThread();
    clear(group);   // 先移除旧对象,保证“整组替换”期间不残留旧帧
    GroupState state;
    state.objects = std::move(objects);
    state.options = options;
    state.visible = visible;
    for (const Handle(AIS_InteractiveObject)& object : state.objects) {
        if (visible)
            displayObject(object, options);
        else
            m_sink->erase(object);
    }
    m_groups.insert_or_assign(group, std::move(state));
}

void OverlayController::append(
    OverlayGroup group,
    Handle(AIS_InteractiveObject) object,
    OverlayDisplayOptions options)
{
    requireOwnerThread();
    if (object.IsNull())
        return;
    auto [found, inserted] = m_groups.try_emplace(group);
    GroupState& state = found->second;
    if (inserted) {
        state.options = options;
        state.visible = true;
    }
    try {
        if (state.visible)
            displayObject(object, options);
        else
            m_sink->erase(object);
        state.objects.push_back(object);
    } catch (...) {
        m_sink->remove(object);
        if (state.objects.empty())
            m_groups.erase(found);
        throw;
    }
}

void OverlayController::setVisible(OverlayGroup group, bool visible)
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end() || found->second.visible == visible)
        return;
    found->second.visible = visible;
    for (const Handle(AIS_InteractiveObject)& object : found->second.objects) {
        if (visible)
            displayObject(object, found->second.options);
        else
            m_sink->erase(object);
    }
}

void OverlayController::setLocalTransformation(
    OverlayGroup group, const gp_Trsf& transform)
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end())
        return;
    for (const Handle(AIS_InteractiveObject)& object : found->second.objects) {
        if (!object.IsNull())
            object->SetLocalTransformation(transform);
    }
}

void OverlayController::setDisplayPriority(
    OverlayGroup group, std::size_t objectIndex, int priority)
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end() ||
        objectIndex >= found->second.objects.size()) {
        return;
    }
    m_sink->setDisplayPriority(
        found->second.objects[objectIndex], priority);
}

void OverlayController::redisplay(OverlayGroup group)
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end() || !found->second.visible)
        return;
    for (const Handle(AIS_InteractiveObject)& object : found->second.objects) {
        if (m_sink->isDisplayed(object))
            m_sink->redisplay(object);
        else
            displayObject(object, found->second.options);
    }
}

void OverlayController::clear(OverlayGroup group)
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end())
        return;
    for (const Handle(AIS_InteractiveObject)& object : found->second.objects)
        m_sink->remove(object);
    m_groups.erase(found);
}

void OverlayController::clearAllTransient()
{
    requireOwnerThread();
    while (!m_groups.empty())
        clear(m_groups.begin()->first);
}

Handle(AIS_InteractiveObject) OverlayController::first(
    OverlayGroup group) const
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end() || found->second.objects.empty())
        return {};
    return found->second.objects.front();
}

Handle(AIS_InteractiveObject) OverlayController::at(
    OverlayGroup group, std::size_t objectIndex) const
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    if (found == m_groups.end() ||
        objectIndex >= found->second.objects.size()) {
        return {};
    }
    return found->second.objects[objectIndex];
}

std::size_t OverlayController::size(OverlayGroup group) const
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    return found == m_groups.end() ? 0 : found->second.objects.size();
}

bool OverlayController::visible(OverlayGroup group) const
{
    requireOwnerThread();
    const auto found = m_groups.find(group);
    return found != m_groups.end() && found->second.visible;
}

} // namespace oap::app::presentation
