// 顺序材料去除的在制毛坯状态。该层只负责验证并应用调用方提供的切削体，
// 不理解具体刀路策略；孔、槽腔和后续机床仿真可共享同一状态模型。
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace oap::toolpath {

// 单次减料结果状态：Applied 表示成功提交；其余为取消或各类失败。
// 失败时在制毛坯保持上一个有效快照不变。
enum class StockRemovalStatus {
    Applied,
    NoMaterialRemoved,
    InvalidStock,
    InvalidRemovalVolume,
    BooleanFailed,
    InvalidResult,
    Cancelled,
};

// 减料执行的阶段，供回调做阶段级进度/忙碌提示。
enum class StockRemovalPhase {
    ValidatingRemovalVolume,
    ApplyingBooleanRemoval,
    ValidatingRemovalResult,
};

using StockRemovalPhaseCallback =
    std::function<void(StockRemovalPhase phase)>;

// 单步去料记录：工序、状态与去料前后体积。
struct StockRemovalRecord {
    std::string operationId;
    int sequenceIndex = -1;
    StockRemovalStatus status = StockRemovalStatus::InvalidStock;
    double volumeBefore = 0.0;
    double volumeAfter = 0.0;
    double removedVolume = 0.0;
    double inputValidationMilliseconds = 0.0;
    double booleanMilliseconds = 0.0;
    double resultValidationMilliseconds = 0.0;
    int booleanBatchCount = 0;
    int sweepSegmentCount = 0;
    int removalSolidCount = 0;
    int planarSweepLayerCount = 0;
    double planarFootprintArea = 0.0;
    double maximumPlanarFootprintArea = 0.0;
    int entrySweepGroupCount = 0;
    int containedEntrySweepGroupCount = 0;
    int containedEntrySweepSegmentCount = 0;
    int fallbackSweepSegmentCount = 0;
    std::string message;
    // OCCT 形状是共享句柄语义；保存快照不会复制底层全部拓扑数据。
    TopoDS_Shape stockAfter;
};

const char* stockRemovalStatusName(StockRemovalStatus status) noexcept;

// 在制毛坯：顺序材料去除的状态机。构造时校验初始毛坯；每次 applyRemoval
// 从当前形状减去切削体并记录历史快照，失败时保持原状。非线程安全，
// 同一实例需由单线程串行驱动。
class InProcessStock {
public:
    // 构造：校验初始毛坯为有效正体积实体；geometricTolerance 低于 OCCT
    // Confusion 时自动抬升。runParallel 控制布尔并行度，cancellationRequested
    // 用于在 OCCT 长运算中提前取消。
    explicit InProcessStock(const TopoDS_Shape& initialStock,
                            double geometricTolerance = 1.0e-5,
                            bool runParallel = true,
                            std::function<bool()> cancellationRequested = {});

    bool valid() const noexcept { return m_valid; }
    const TopoDS_Shape& initialShape() const noexcept { return m_initial; }
    const TopoDS_Shape& currentShape() const noexcept { return m_current; }
    double initialVolume() const noexcept { return m_initialVolume; }
    double currentVolume() const noexcept { return m_currentVolume; }
    const std::vector<StockRemovalRecord>& history() const noexcept {
        return m_history;
    }

    // 应用一次减料：把 removalVolume 当作整体与当前毛坯相减。
    // 返回记录含体积与耗时统计；失败时当前形状不变。
    StockRemovalRecord applyRemoval(const TopoDS_Shape& removalVolume,
                                    std::string operationId,
                                    int sequenceIndex,
                                    StockRemovalPhaseCallback phaseChanged = {});
    // 槽腔线性扫掠由大量相互重叠的独立实体组成。将其展开为 OCCT 多工具
    // 分批相减，避免嵌套 Compound 被当作单一工具而产生空切或单次规模失控。
    StockRemovalRecord applyRemovalWithIndependentSolids(
        const TopoDS_Shape& removalVolume,
        std::string operationId,
        int sequenceIndex,
        StockRemovalPhaseCallback phaseChanged = {});
    // 复位到初始毛坯并清空历史记录。
    void reset();

private:
    TopoDS_Shape m_initial;
    TopoDS_Shape m_current;
    double m_tolerance = 1.0e-5;
    double m_initialVolume = 0.0;
    double m_currentVolume = 0.0;
    bool m_valid = false;
    bool m_runParallel = true;
    std::function<bool()> m_cancellationRequested;
    std::vector<StockRemovalRecord> m_history;

    StockRemovalRecord applyRemovalImpl(
        const TopoDS_Shape& removalVolume,
        std::string operationId,
        int sequenceIndex,
        bool independentSolids,
        StockRemovalPhaseCallback phaseChanged);
};

} // namespace oap::toolpath
