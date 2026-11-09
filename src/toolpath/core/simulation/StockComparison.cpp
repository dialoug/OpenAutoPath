// 通过 Common/Cut 组合区分初始缺料、最终余量和真正过切。
#include "toolpath/core/simulation/StockComparison.h"
#include "toolpath/core/ToolpathOcctParallel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>

namespace oap::toolpath {
namespace {

using SteadyClock = std::chrono::steady_clock;

class CancellationProgressIndicator final
    : public Message_ProgressIndicator {
public:
    explicit CancellationProgressIndicator(
        std::function<bool()> cancellationRequested)
        : m_cancellationRequested(
              std::move(cancellationRequested))
    {
    }

protected:
    Standard_Boolean UserBreak() override
    {
        try {
            return m_cancellationRequested &&
                   m_cancellationRequested();
        } catch (...) {
            return Standard_True;
        }
    }

    void Show(const Message_ProgressScope&,
              const Standard_Boolean) override
    {
    }

private:
    std::function<bool()> m_cancellationRequested;
};

template <typename BooleanAlgorithm>
void buildBoolean(
    BooleanAlgorithm& algorithm,
    const std::function<bool()>& cancellationRequested)
{
    if (!cancellationRequested) {
        algorithm.Build();
        return;
    }
    const Handle(Message_ProgressIndicator) progress =
        new CancellationProgressIndicator(cancellationRequested);
    algorithm.Build(progress->Start());
}

double elapsedMilliseconds(const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

class ScopedElapsedMilliseconds {
public:
    explicit ScopedElapsedMilliseconds(double& output)
        : m_output(output), m_start(SteadyClock::now())
    {
    }

    ~ScopedElapsedMilliseconds()
    {
        m_output = elapsedMilliseconds(m_start);
    }

private:
    double& m_output;
    SteadyClock::time_point m_start;
};

bool validShape(const TopoDS_Shape& shape, bool runParallel)
{
    if (shape.IsNull())
        return true;
    try {
        ToolpathOcctParallelLease lease(runParallel);
        return BRepCheck_Analyzer(
                   shape, Standard_True, lease.enabled())
            .IsValid();
    } catch (...) {
        return false;
    }
}

bool shapeVolume(const TopoDS_Shape& shape, double& volume)
{
    if (shape.IsNull()) {
        volume = 0.0;
        return true;
    }
    try {
        GProp_GProps properties;
        BRepGProp::VolumeProperties(shape, properties);
        volume = std::abs(properties.Mass());
        return std::isfinite(volume);
    } catch (...) {
        return false;
    }
}

bool cutShape(const TopoDS_Shape& argument,
              const TopoDS_Shape& tool,
              double tolerance,
              bool runParallel,
              const std::function<bool()>& cancellationRequested,
              TopoDS_Shape& result)
{
    if (argument.IsNull()) {
        result.Nullify();
        return true;
    }
    if (tool.IsNull()) {
        result = argument;
        return true;
    }
    try {
        ToolpathOcctParallelLease lease(runParallel);
        // Do not run the eager two-shape constructor before the configured,
        // cancellable Build: that would compute the same difference twice.
        BRepAlgoAPI_Cut cut;
        TopTools_ListOfShape arguments, tools;
        arguments.Append(argument);
        tools.Append(tool);
        cut.SetArguments(arguments);
        cut.SetTools(tools);
        cut.SetFuzzyValue(tolerance);
        cut.SetRunParallel(lease.enabled());
        buildBoolean(cut, cancellationRequested);
        if (!cut.IsDone())
            return false;
        result = cut.Shape();
        return validShape(result, runParallel);
    } catch (...) {
        return false;
    }
}

bool commonShape(const TopoDS_Shape& first,
                 const TopoDS_Shape& second,
                 double tolerance,
                 bool runParallel,
                 const std::function<bool()>& cancellationRequested,
                 TopoDS_Shape& result)
{
    if (first.IsNull() || second.IsNull()) {
        result.Nullify();
        return true;
    }
    try {
        ToolpathOcctParallelLease lease(runParallel);
        BRepAlgoAPI_Common common;
        TopTools_ListOfShape arguments, tools;
        arguments.Append(first);
        tools.Append(second);
        common.SetArguments(arguments);
        common.SetTools(tools);
        common.SetFuzzyValue(tolerance);
        common.SetRunParallel(lease.enabled());
        buildBoolean(common, cancellationRequested);
        if (!common.IsDone())
            return false;
        result = common.Shape();
        return validShape(result, runParallel);
    } catch (...) {
        return false;
    }
}

struct CutAndVolumeResult {
    TopoDS_Shape shape;
    double volume = 0.0;
    double milliseconds = 0.0;
    bool cutSucceeded = false;
    bool volumeMeasured = false;
};

CutAndVolumeResult cutAndMeasure(
    const TopoDS_Shape& argument,
    const TopoDS_Shape& tool,
    double tolerance,
    bool runParallel,
    const std::function<bool()>& cancellationRequested)
{
    CutAndVolumeResult result;
    const SteadyClock::time_point start =
        SteadyClock::now();
    result.cutSucceeded = cutShape(
        argument, tool, tolerance, runParallel,
        cancellationRequested, result.shape);
    if (result.cutSucceeded) {
        result.volumeMeasured =
            shapeVolume(result.shape, result.volume);
    }
    result.milliseconds =
        elapsedMilliseconds(start);
    return result;
}

bool shapeBounds(const TopoDS_Shape& shape,
                 std::array<double, 6>& values)
{
    if (shape.IsNull())
        return false;
    try {
        Bnd_Box bounds;
        BRepBndLib::AddOptimal(shape, bounds, Standard_False, Standard_False);
        if (bounds.IsVoid())
            return false;
        bounds.Get(values[0], values[1], values[2],
                   values[3], values[4], values[5]);
        return std::all_of(values.begin(), values.end(),
                           [](double value) { return std::isfinite(value); });
    } catch (...) {
        return false;
    }
}

void addDiagnostic(StockComparisonResult& result,
                   ToolpathDiagnosticCode code,
                   ToolpathDiagnosticSeverity severity,
                   std::string message)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    result.diagnostics.push_back(std::move(diagnostic));
}

void reportExecutionStatus(
    const ToolpathGenerationControl* control,
    ToolpathExecutionPhase phase,
    const std::string& operationId = {},
    std::size_t operationIndex = 0,
    std::size_t operationCount = 0)
{
    if (!control)
        return;
    ToolpathExecutionStatus status;
    status.phase = phase;
    status.operationId = operationId;
    status.operationIndex = operationIndex;
    status.operationCount = operationCount;
    control->status(std::move(status));
}

} // namespace

TopoDS_Shape buildHoleFeatureAllowanceRegion(
    const gp_Pnt& entry,
    const gp_Pnt& bottom,
    double radius,
    double entryExtension,
    double bottomExtension)
{
    const gp_Vec axis(entry, bottom);
    const double nominalDepth = axis.Magnitude();
    if (!std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(nominalDepth) ||
        nominalDepth <= Precision::Confusion()) {
        return {};
    }
    entryExtension = std::max(0.0, entryExtension);
    bottomExtension = std::max(0.0, bottomExtension);
    if (!std::isfinite(entryExtension) ||
        !std::isfinite(bottomExtension)) {
        return {};
    }
    try {
        const gp_Dir direction(axis);
        const gp_Pnt start = entry.Translated(
            gp_Vec(direction).Multiplied(-entryExtension));
        return BRepPrimAPI_MakeCylinder(
            gp_Ax2(start, direction), radius,
            nominalDepth + entryExtension + bottomExtension).Shape();
    } catch (...) {
        return {};
    }
}

TopoDS_Shape buildHoleFeatureAllowanceRegion(
    const RecognizedHoleGeometry& hole,
    double boundaryMargin)
{
    boundaryMargin = std::max(0.0, boundaryMargin);
    const double bottomExtension =
        hole.bottomProfile == HoleBottomProfile::Conical
        ? std::max(boundaryMargin, hole.radius)
        : boundaryMargin;
    return buildHoleFeatureAllowanceRegion(
        hole.entry, hole.bottom, hole.radius,
        boundaryMargin, bottomExtension);
}

// 三阶段精确差异：
// 1) 目标减初始毛坯 → 初始缺料；
// 2) 最终毛坯减目标 → 剩余加工余量；
// 3) 初始存在且最终缺失的目标材料 → 真正过切。
// 各阶段独立布尔、独立计时，任一阶段失败即整体失败并返回已完成部分。
StockComparisonResult StockComparison::compare(
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& finalStock,
    const TopoDS_Shape& targetShape,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    StockComparisonResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    const double tolerance = std::isfinite(options.geometricTolerance)
        ? std::max(options.geometricTolerance, Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    std::function<bool()> cancellationCheck;
    if (control) {
        cancellationCheck = [control] {
            return control->cancelled();
        };
    }
    if (control)
        control->progress(0, ProgressStepCount);
    reportExecutionStatus(
        control,
        ToolpathExecutionPhase::
            ValidatingComparisonInputs);
    if (control && control->cancelled()) {
        result.cancelled = true;
        addDiagnostic(
            result, ToolpathDiagnosticCode::GenerationCancelled,
            ToolpathDiagnosticSeverity::Warning,
            "毛坯差异检查在校验前已取消");
        return result;
    }

    const SteadyClock::time_point inputValidationStart =
        SteadyClock::now();
    const bool stockShapesValid =
        options.initialAndFinalShapesPrevalidated ||
        (validShape(initialStock, options.runParallel) &&
         validShape(finalStock, options.runParallel));
    if (initialStock.IsNull() || targetShape.IsNull() ||
        !stockShapesValid ||
        !validShape(targetShape, options.runParallel) ||
        !shapeVolume(initialStock, result.initialVolume) ||
        !shapeVolume(finalStock, result.finalVolume) ||
        !shapeVolume(targetShape, result.targetVolume) ||
        result.initialVolume <= 0.0 || result.targetVolume <= 0.0) {
        addDiagnostic(result, ToolpathDiagnosticCode::StockComparisonFailed,
                      ToolpathDiagnosticSeverity::Error,
                      "毛坯差异检查要求初始毛坯、最终毛坯和目标均为有效正体积形体");
        result.inputValidationMilliseconds =
            elapsedMilliseconds(
                inputValidationStart);
        return result;
    }

    std::array<double, 6> initialBounds{};
    std::array<double, 6> targetBounds{};
    result.hasBounds = shapeBounds(initialStock, initialBounds) &&
                       shapeBounds(targetShape, targetBounds);
    if (result.hasBounds) {
        for (int axis = 0; axis < 3; ++axis) {
            result.initialBoundsCenter[axis] =
                0.5 * (initialBounds[axis] + initialBounds[axis + 3]);
            result.targetBoundsCenter[axis] =
                0.5 * (targetBounds[axis] + targetBounds[axis + 3]);
            result.boundsCenterOffset[axis] =
                result.targetBoundsCenter[axis] - result.initialBoundsCenter[axis];
        }
        result.targetBoundsInsideInitialBounds =
            targetBounds[0] >= initialBounds[0] - tolerance &&
            targetBounds[1] >= initialBounds[1] - tolerance &&
            targetBounds[2] >= initialBounds[2] - tolerance &&
            targetBounds[3] <= initialBounds[3] + tolerance &&
            targetBounds[4] <= initialBounds[4] + tolerance &&
            targetBounds[5] <= initialBounds[5] + tolerance;
    }
    result.inputValidationMilliseconds =
        elapsedMilliseconds(
            inputValidationStart);
    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({
            1.0,
            result.initialVolume,
            result.targetVolume}));

    auto cancelled = [&]() {
        if (!control || !control->cancelled())
            return false;
        result.cancelled = true;
        addDiagnostic(result, ToolpathDiagnosticCode::GenerationCancelled,
                      ToolpathDiagnosticSeverity::Warning,
                      "毛坯差异检查已取消，已保留完成的差异形体");
        return true;
    };
    auto failed = [&](const char* operation) {
        addDiagnostic(result, ToolpathDiagnosticCode::StockComparisonFailed,
                      ToolpathDiagnosticSeverity::Error,
                      std::string("毛坯差异布尔运算失败：") + operation);
    };
    auto progress = [&](std::size_t step) {
        if (control)
            control->progress(step, ProgressStepCount);
    };

    if (cancelled())
        return result;
    reportExecutionStatus(
        control,
        ToolpathExecutionPhase::
            ComputingInitialMissing);
    const SteadyClock::time_point
        initialMissingStart =
            SteadyClock::now();
    const bool initialMissingSucceeded =
        cutShape(
            targetShape, initialStock, tolerance,
            options.runParallel,
            cancellationCheck,
            result.initialMissing);
    if (!initialMissingSucceeded) {
        result.initialMissingMilliseconds =
            elapsedMilliseconds(
                initialMissingStart);
        if (cancelled())
            return result;
        failed("目标实体减初始毛坯");
        return result;
    }
    progress(1);
    if (!shapeVolume(result.initialMissing, result.initialMissingVolume)) {
        failed("测量初始缺料体积");
        return result;
    }
    result.initialMissingMilliseconds =
        elapsedMilliseconds(
            initialMissingStart);
    result.initialContainsTarget =
        result.initialMissingVolume <=
        volumeTolerance;

    auto storeDifference =
        [&](CutAndVolumeResult&& difference,
            TopoDS_Shape& shape,
            double& volume,
            double& milliseconds,
            const char* booleanOperation,
            const char* volumeOperation) {
            milliseconds = difference.milliseconds;
            if (!difference.cutSucceeded) {
                if (!cancelled())
                    failed(booleanOperation);
                return false;
            }
            if (!difference.volumeMeasured) {
                failed(volumeOperation);
                return false;
            }
            shape = std::move(difference.shape);
            volume = difference.volume;
            return true;
        };

    if (result.initialContainsTarget &&
        options.runParallel) {
        // 目标已由精确差集证明完整位于初始毛坯中。此时剩余余量
        // (final-target) 与过切 (target-final) 完全独立，可并发执行；
        // 两个分支仍调用相同的精确 OCCT Cut，并保持原有结果语义。
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::
                ComputingRemainingAllowance);
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::ComputingOvercut);
        // Parallelize one layer only: at most two budgeted independent cuts,
        // with internal OCCT parallelism disabled. Zero slots run in the caller.
        std::array<CutAndVolumeResult, 2> differences;
        toolpathParallelForBudgeted(2u, [&](std::size_t index) {
            OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
            differences[index] = index == 0u
                ? cutAndMeasure(finalStock, targetShape, tolerance, false, cancellationCheck)
                : cutAndMeasure(targetShape, finalStock, tolerance, false, cancellationCheck);
        }, control);
        CutAndVolumeResult remainingDifference = std::move(differences[0]);
        CutAndVolumeResult overcutDifference = std::move(differences[1]);

        if (!storeDifference(
                std::move(remainingDifference),
                result.remainingAllowance,
                result.remainingAllowanceVolume,
                result.remainingAllowanceMilliseconds,
                "最终毛坯减目标实体",
                "测量剩余加工余量体积")) {
            return result;
        }
        progress(2);

        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::
                IntersectingInitialTarget);
        result.initialTargetIntersectionMilliseconds =
            0.0;
        progress(3);

        if (!storeDifference(
                std::move(overcutDifference),
                result.overcut,
                result.overcutVolume,
                result.overcutMilliseconds,
                "初始存在的目标材料减最终毛坯",
                "测量过切体积")) {
            return result;
        }
        progress(4);
    } else {
        if (cancelled())
            return result;
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::
                ComputingRemainingAllowance);
        if (!storeDifference(
                cutAndMeasure(
                    finalStock, targetShape,
                    tolerance, options.runParallel,
                    cancellationCheck),
                result.remainingAllowance,
                result.remainingAllowanceVolume,
                result.remainingAllowanceMilliseconds,
                "最终毛坯减目标实体",
                "测量剩余加工余量体积")) {
            return result;
        }
        progress(2);

        if (cancelled())
            return result;
        TopoDS_Shape initiallyPresentTarget;
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::
                IntersectingInitialTarget);
        const SteadyClock::time_point
            initialTargetIntersectionStart =
                SteadyClock::now();
        if (result.initialContainsTarget) {
            // 第一步精确差集已经证明目标完全位于初始毛坯内；此时交集
            // 必然等于目标本身，复用目标可避免再次执行 PaveFiller。
            initiallyPresentTarget = targetShape;
        } else if (!commonShape(
                       initialStock, targetShape,
                       tolerance,
                       options.runParallel,
                       cancellationCheck,
                       initiallyPresentTarget)) {
            result.initialTargetIntersectionMilliseconds =
                elapsedMilliseconds(
                    initialTargetIntersectionStart);
            if (cancelled())
                return result;
            failed("初始毛坯与目标实体求交");
            return result;
        }
        result.initialTargetIntersectionMilliseconds =
            elapsedMilliseconds(
                initialTargetIntersectionStart);
        progress(3);

        if (cancelled())
            return result;
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::ComputingOvercut);
        if (!storeDifference(
                cutAndMeasure(
                    initiallyPresentTarget, finalStock,
                    tolerance, options.runParallel,
                    cancellationCheck),
                result.overcut,
                result.overcutVolume,
                result.overcutMilliseconds,
                "初始存在的目标材料减最终毛坯",
                "测量过切体积")) {
            return result;
        }
        progress(4);
    }
    result.targetContainmentRatio = std::clamp(
        1.0 - result.initialMissingVolume / result.targetVolume, 0.0, 1.0);
    if (!result.initialContainsTarget) {
        addDiagnostic(result, ToolpathDiagnosticCode::StockTargetOutsideInitial,
                      ToolpathDiagnosticSeverity::Error,
                      "初始毛坯缺少部分目标材料");
    }
    if (result.remainingAllowanceVolume > volumeTolerance) {
        addDiagnostic(result, ToolpathDiagnosticCode::StockRemainingAllowance,
                      ToolpathDiagnosticSeverity::Info,
                      "当前工序完成后目标实体外仍有剩余加工余量");
    }
    if (result.overcutVolume > volumeTolerance) {
        addDiagnostic(result, ToolpathDiagnosticCode::StockOvercutDetected,
                      ToolpathDiagnosticSeverity::Error,
                      "当前工序去除了目标实体内部应保留的材料，体积 " +
                          std::to_string(result.overcutVolume) +
                          " mm³");
    }
    result.complete = true;
    if (control)
        control->status({});
    return result;
}

StockOvercutAttributionResult StockComparison::attributeOvercut(
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    const std::vector<StockRemovalRecord>& steps,
    const StockComparisonOptions& options,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    StockOvercutAttributionResult result;
    ScopedElapsedMilliseconds totalTimer(
        result.totalMilliseconds);
    const double tolerance = std::isfinite(options.geometricTolerance)
        ? std::max(options.geometricTolerance, Precision::Confusion())
        : std::max(1.0e-5, Precision::Confusion());
    std::function<bool()> cancellationCheck;
    if (control) {
        cancellationCheck = [control] {
            return control->cancelled();
        };
    }
    if (control)
        control->progress(0, steps.size());
    reportExecutionStatus(
        control,
        ToolpathExecutionPhase::AttributingOvercut,
        {}, 0, steps.size());
    if (control && control->cancelled()) {
        result.cancelled = true;
        ToolpathDiagnostic diagnostic;
        diagnostic.code =
            ToolpathDiagnosticCode::GenerationCancelled;
        diagnostic.severity =
            ToolpathDiagnosticSeverity::Warning;
        diagnostic.message =
            "过切归因在校验前已取消";
        result.diagnostics.push_back(std::move(diagnostic));
        return result;
    }

    double initialVolume = 0.0;
    double targetVolume = 0.0;
    if (initialStock.IsNull() || targetShape.IsNull() ||
        !validShape(initialStock, options.runParallel) ||
        !validShape(targetShape, options.runParallel) ||
        !shapeVolume(initialStock, initialVolume) ||
        !shapeVolume(targetShape, targetVolume) ||
        initialVolume <= 0.0 || targetVolume <= 0.0) {
        ToolpathDiagnostic diagnostic;
        diagnostic.code = ToolpathDiagnosticCode::StockComparisonFailed;
        diagnostic.severity = ToolpathDiagnosticSeverity::Error;
        diagnostic.message =
            "过切归因要求初始毛坯和目标均为有效正体积形体";
        result.diagnostics.push_back(std::move(diagnostic));
        return result;
    }
    const double volumeTolerance = std::max(
        tolerance * tolerance * tolerance,
        1.0e-9 * std::max({1.0, initialVolume, targetVolume}));

    auto reportFailure = [&](const std::string& operationId,
                             const char* message) {
        ToolpathDiagnostic diagnostic;
        diagnostic.code = ToolpathDiagnosticCode::StockComparisonFailed;
        diagnostic.severity = ToolpathDiagnosticSeverity::Error;
        diagnostic.message = message;
        diagnostic.operationId = operationId;
        result.diagnostics.push_back(std::move(diagnostic));
    };
    auto cancelled = [&]() {
        if (!control || !control->cancelled())
            return false;
        result.cancelled = true;
        ToolpathDiagnostic diagnostic;
        diagnostic.code = ToolpathDiagnosticCode::GenerationCancelled;
        diagnostic.severity = ToolpathDiagnosticSeverity::Warning;
        diagnostic.message =
            "过切归因已取消，已保留完成的工序归因结果";
        result.diagnostics.push_back(std::move(diagnostic));
        return true;
    };

    TopoDS_Shape presentBefore;
    if (!commonShape(
            initialStock, targetShape, tolerance,
            options.runParallel, cancellationCheck,
            presentBefore)) {
        if (cancelled())
            return result;
        reportFailure({}, "过切归因无法计算初始毛坯与目标实体的交集");
        return result;
    }
    TopoDS_Shape finalStock = initialStock;
    for (const StockRemovalRecord& step : steps) {
        if (step.status == StockRemovalStatus::Applied)
            finalStock = step.stockAfter;
    }
    TopoDS_Shape remainingConfirmedOvercut;
    if (!cutShape(
            presentBefore, finalStock, tolerance,
            options.runParallel, cancellationCheck,
            remainingConfirmedOvercut)) {
        if (cancelled())
            return result;
        reportFailure(
            {},
            "过切归因无法计算最终毛坯中确认缺失的目标材料");
        return result;
    }

    for (std::size_t index = 0; index < steps.size(); ++index) {
        if (cancelled())
            return result;
        const StockRemovalRecord& step = steps[index];
        if (step.status != StockRemovalStatus::Applied) {
            if (control)
                control->progress(index + 1, steps.size());
            continue;
        }
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::AttributingOvercut,
            step.operationId, index, steps.size());

        // 直接从“上一步仍存在的目标材料”中减去当前毛坯。这样得到的
        // removedCandidate 天然属于目标材料，且不会像分别计算
        // Common(stockBefore, target) / Common(stockAfter, target) 那样
        // 因复杂 BRep 的独立布尔分割产生非单调体积波动。
        TopoDS_Shape removedCandidate;
        if (!cutShape(
                presentBefore, step.stockAfter, tolerance,
                options.runParallel,
                cancellationCheck, removedCandidate)) {
            if (cancelled())
                return result;
            reportFailure(step.operationId,
                          "无法计算该工序去除的目标材料");
            return result;
        }
        // 每道工序只允许领取最终差异检查已经确认的过切区域，并从
        // remainingConfirmedOvercut 中扣除，防止数值薄片被多次归因。
        TopoDS_Shape removedTarget;
        if (!commonShape(
                removedCandidate,
                remainingConfirmedOvercut,
                tolerance,
                options.runParallel,
                cancellationCheck, removedTarget)) {
            if (cancelled())
                return result;
            reportFailure(
                step.operationId,
                "无法把该工序去除量限制到最终确认过切区域");
            return result;
        }
        double removedTargetVolume = 0.0;
        if (!shapeVolume(
                removedTarget, removedTargetVolume)) {
            reportFailure(step.operationId,
                          "无法测量该工序去除的目标材料体积");
            return result;
        }

        TopoDS_Shape presentAfter;
        if (!cutShape(
                presentBefore, removedCandidate, tolerance,
                options.runParallel,
                cancellationCheck, presentAfter)) {
            if (cancelled())
                return result;
            reportFailure(step.operationId,
                          "无法更新工序完成后仍存在的目标材料");
            return result;
        }
        if (removedTargetVolume > volumeTolerance) {
            StockOperationOvercut attribution;
            attribution.operationId = step.operationId;
            attribution.sequenceIndex = step.sequenceIndex;
            attribution.shape = removedTarget;
            attribution.volume = removedTargetVolume;
            result.attributedVolume += attribution.volume;
            result.operations.push_back(std::move(attribution));

            ToolpathDiagnostic diagnostic;
            diagnostic.code = ToolpathDiagnosticCode::StockOvercutDetected;
            diagnostic.severity = ToolpathDiagnosticSeverity::Error;
            diagnostic.message =
                "工序 " + step.operationId +
                " 去除了目标实体中应保留的材料，体积 " +
                std::to_string(removedTargetVolume) +
                " mm³";
            diagnostic.operationId = step.operationId;
            result.diagnostics.push_back(std::move(diagnostic));
        }

        if (!removedTarget.IsNull()) {
            TopoDS_Shape remainingAfter;
            if (!cutShape(
                    remainingConfirmedOvercut,
                    removedTarget,
                    tolerance,
                    options.runParallel,
                    cancellationCheck,
                    remainingAfter)) {
                if (cancelled())
                    return result;
                reportFailure(
                    step.operationId,
                    "无法更新尚未归因的最终过切区域");
                return result;
            }
            remainingConfirmedOvercut =
                std::move(remainingAfter);
        }

        presentBefore = presentAfter;
        if (control)
            control->progress(index + 1, steps.size());
    }
    double unattributedVolume = 0.0;
    if (!shapeVolume(
            remainingConfirmedOvercut,
            unattributedVolume)) {
        reportFailure({}, "无法测量尚未归因的最终过切体积");
        return result;
    }
    if (unattributedVolume > volumeTolerance) {
        ToolpathDiagnostic diagnostic;
        diagnostic.code =
            ToolpathDiagnosticCode::StockComparisonFailed;
        diagnostic.severity =
            ToolpathDiagnosticSeverity::Warning;
        diagnostic.message =
            "最终确认过切中仍有 " +
            std::to_string(unattributedVolume) +
            " mm³ 未能稳定归因到单道工序";
        result.diagnostics.push_back(
            std::move(diagnostic));
    }
    result.complete = true;
    if (control)
        control->status({});
    return result;
}

void analyzeFeatureAllowances(
    const TopoDS_Shape& initialStock,
    const TopoDS_Shape& targetShape,
    const std::vector<StockFeatureAllowanceScope>& scopes,
    StockComparisonResult& result)
{
    result.featureAllowances.clear();
    if (scopes.empty() || initialStock.IsNull() ||
        targetShape.IsNull() || !result.complete) {
        return;
    }

    constexpr double tolerance = 1.0e-5;
    TopoDS_Shape initialRemovalMaterial;
    if (!cutShape(
            initialStock, targetShape, tolerance, true, {},
            initialRemovalMaterial)) {
        return;
    }

    result.featureAllowances.reserve(scopes.size());
    for (const StockFeatureAllowanceScope& scope : scopes) {
        if (scope.region.IsNull() || scope.featureId.empty())
            continue;
        TopoDS_Shape targetRemoval;
        TopoDS_Shape remaining;
        double targetVolume = 0.0;
        double remainingVolume = 0.0;
        if (!commonShape(
                initialRemovalMaterial, scope.region,
                tolerance, true, {}, targetRemoval) ||
            !shapeVolume(targetRemoval, targetVolume) ||
            !commonShape(
                result.remainingAllowance, scope.region,
                tolerance, true, {}, remaining) ||
            !shapeVolume(remaining, remainingVolume)) {
            continue;
        }
        if (targetVolume <= tolerance * tolerance * tolerance)
            continue;

        StockFeatureAllowance allowance;
        allowance.featureId = scope.featureId;
        allowance.operationId = scope.operationId;
        allowance.featureKind = scope.featureKind;
        allowance.sourceIndex = scope.sourceIndex;
        allowance.targetRemovalVolume = targetVolume;
        allowance.remainingAllowanceVolume = std::clamp(
            remainingVolume, 0.0, targetVolume);
        allowance.completionRatio = std::clamp(
            1.0 - allowance.remainingAllowanceVolume / targetVolume,
            0.0, 1.0);
        allowance.approximateBoundary = scope.approximateBoundary;
        allowance.remainingAllowance = remaining;
        result.featureAllowances.push_back(std::move(allowance));
    }
}

} // namespace oap::toolpath
