// AIS overlay 的语义分组与唯一注册表；所有 sink 调用必须发生在 owner 线程。
#pragma once

#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <gp_Trsf.hxx>

#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace oap::app::presentation {

// 覆盖组语义分类:一组相关 AIS 对象作为一个整体被替换/清除/切换可见性。
enum class OverlayGroup {
    Toolpath,           // 完整刀路预览
    CurrentToolpath,    // 播放中的当前刀路段
    PlaybackRemainingPath, // 播放时刀尖之后的剩余路径
    PlaybackTrail,      // 播放轨迹
    PlaybackCursor,     // 刀尖光标(球 + 点)
    HoleStock,          // 孔毛坯
    PocketIpw,          // 腔 IPW(三角化/图元/BRep)
    TemporaryHighlight, // 检查用临时高亮(面/见证线/包围盒)
    HoleStageHighlight, // 孔阶段壁面与切削路径高亮
};

// 单个覆盖对象的显示选项:deactivateSelection 控制显示后是否停用拾取;
// displayPriority < 0 表示不设置显示优先级。
struct OverlayDisplayOptions {
    bool deactivateSelection = true;
    int displayPriority = -1;
};

// 小型显示端口使所有权/替换逻辑可在不创建 Viewer 的测试中验证。
class PresentationSink {
public:
    virtual ~PresentationSink() = default;
    // 原生 OCCT 查看器在主窗口显示之后才创建,因此 sink 必须支持
    // 替换构造期间捕获的 context。测试用的 sink 可保留默认的空实现。
    virtual void bindContext(
        const Handle(AIS_InteractiveContext)&) {}
    virtual void display(const Handle(AIS_InteractiveObject)& object) = 0;
    virtual void erase(const Handle(AIS_InteractiveObject)& object) = 0;
    virtual void remove(const Handle(AIS_InteractiveObject)& object) = 0;
    virtual void redisplay(const Handle(AIS_InteractiveObject)& object) = 0;
    virtual void deactivate(const Handle(AIS_InteractiveObject)& object) = 0;
    virtual void setDisplayPriority(
        const Handle(AIS_InteractiveObject)& object, int priority) = 0;
    virtual bool isDisplayed(
        const Handle(AIS_InteractiveObject)& object) const = 0;
};

// OCCT 呈现接收器：把 display/erase/remove 等呈现请求应用到 AIS 上下文。
class OcctPresentationSink final : public PresentationSink {
public:
    explicit OcctPresentationSink(
        Handle(AIS_InteractiveContext) context);

    void bindContext(
        const Handle(AIS_InteractiveContext)& context) override;
    void display(const Handle(AIS_InteractiveObject)& object) override;
    void erase(const Handle(AIS_InteractiveObject)& object) override;
    void remove(const Handle(AIS_InteractiveObject)& object) override;
    void redisplay(const Handle(AIS_InteractiveObject)& object) override;
    void deactivate(const Handle(AIS_InteractiveObject)& object) override;
    void setDisplayPriority(
        const Handle(AIS_InteractiveObject)& object, int priority) override;
    bool isDisplayed(
        const Handle(AIS_InteractiveObject)& object) const override;

private:
    Handle(AIS_InteractiveContext) m_context;
};

// overlay 覆盖组的语义化注册表:每个 OverlayGroup 对应一组相关 AIS 对象,
// 以“整组替换 / 追加 / 可见性 / 清除”为粒度的显示事务入口。所有对 sink 的
// 调用必须发生在 owner 线程(构造线程),其余线程调用会抛出 std::logic_error。
//
// 所有权约定:注册进覆盖组的 AIS 句柄由 OverlayController 持有并在 clear /
// 析构时经 sink->remove() 释放;各 Presenter 只借用 Handle 做局部操作,
// 不负责对象的销毁。
class OverlayController {
public:
    // sink:显示端口(生产环境为 OcctPresentationSink),控制器以 shared_ptr
    // 持有,允许在无 Viewer 的测试中注入 mock。不可为空,否则抛出异常。
    explicit OverlayController(std::shared_ptr<PresentationSink> sink);
    ~OverlayController();

    OverlayController(const OverlayController&) = delete;
    OverlayController& operator=(const OverlayController&) = delete;

    // 重新绑定延迟创建的 OCCT context,并发布那些可能在查看器初始化
    // 之前就已注册的可见对象。
    void bindContext(const Handle(AIS_InteractiveContext)& context);
    // 整组替换:先移除组内旧对象,再按 visible 决定显示/擦除新对象,
    // 组选项随本次调用更新。这是“重建一组覆盖对象”的标准入口。
    void replace(
        OverlayGroup group,
        std::vector<Handle(AIS_InteractiveObject)> objects,
        bool visible = true,
        OverlayDisplayOptions options = {});
    // 向已有组追加单个对象(组不存在时创建);对象为空句柄时静默忽略。
    void append(
        OverlayGroup group,
        Handle(AIS_InteractiveObject) object,
        OverlayDisplayOptions options = {});
    // 切换整组可见性:可见则 Display + 按组选项设置,否则 Erase(对象仍留在组中)。
    void setVisible(OverlayGroup group, bool visible);
    // 对组内全部对象统一设置局部变换(模型坐标变换用)。
    void setLocalTransformation(
        OverlayGroup group, const gp_Trsf& transform);
    // 设置组内第 objectIndex 个对象的显示优先级。
    void setDisplayPriority(
        OverlayGroup group, std::size_t objectIndex, int priority);
    // 组可见时对所有已显示对象触发 Redisplay,未显示的对象重新 Display。
    void redisplay(OverlayGroup group);
    // 移除整组:经 sink->remove() 释放组内全部对象并从注册表删除。
    void clear(OverlayGroup group);
    // 清空所有覆盖组(析构时在同一 owner 线程调用)。
    void clearAllTransient();

    Handle(AIS_InteractiveObject) first(OverlayGroup group) const;
    Handle(AIS_InteractiveObject) at(
        OverlayGroup group, std::size_t objectIndex) const;
    std::size_t size(OverlayGroup group) const;
    bool visible(OverlayGroup group) const;

private:
    // 覆盖组状态：组内 AIS 对象与可见性。
    struct GroupState {
        std::vector<Handle(AIS_InteractiveObject)> objects;
        OverlayDisplayOptions options;
        bool visible = true;
    };

    void requireOwnerThread() const;
    void displayObject(
        const Handle(AIS_InteractiveObject)& object,
        const OverlayDisplayOptions& options);

    std::shared_ptr<PresentationSink> m_sink;
    std::thread::id m_ownerThread;
    std::unordered_map<OverlayGroup, GroupState> m_groups;
};

} // namespace oap::app::presentation
