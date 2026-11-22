#include "Simulation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/simulation/FixedAxisToolpathView.h"
#include <algorithm>
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "toolpath/core/PocketToolBinding.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <stdexcept>

namespace oap::worker
{
using namespace manufacturing;
namespace
{
using Stock = std::shared_ptr<const toolpath::MultiDexelStock>;

// Match the desktop transient view: replay from the immutable stock before the
// selected operation, using its bound cutter and the actual hole-tip profile.
Stock moveFrame(const MachiningJob &job, const ipw::IpwNode &node, std::size_t completed, std::size_t &total)
{
    const auto parent = job.runtime().ipwCheckpoints.find(node.parentStateKey);
    const auto *setup = job.setupById(node.operation.setupId);
    if (!parent || !parent->dexelStock || !setup)
        throw std::runtime_error("缺少工序起始毛坯或装夹");
    Stock stock = parent->dexelStock;
    toolpath::ToolpathProgram program;
    toolpath::ToolpathParams params;
    const auto ordered = MachiningOperationService::operations(job.definition());
    const auto formal = std::find_if(ordered.begin(), ordered.end(),
                                     [&](const auto &op) { return op.id == node.operation.id; });
    if (formal == ordered.end())
        throw std::runtime_error("工序不存在");
    switch (formal->kind)
    {
    case JobOperationKind::Hole:
        program = job.holeResult()->toProgram();
        params = job.holeResult()->effectiveParams;
        break;
    case JobOperationKind::Pocket:
        program = job.pocketResult()->toProgram();
        params = job.pocketResult()->effectiveParams;
        break;
    case JobOperationKind::Profile:
        program = job.profileResult()->toProgram();
        params = job.profileResult()->effectiveParams;
        break;
    case JobOperationKind::SurfaceFinishing:
        program = job.currentFinishingResult()->toProgram();
        params = job.currentFinishingResult()->effectiveParams;
        break;
    }
    int sequence = 0;
    std::size_t remaining = completed;
    for (const auto &op : program.operations)
    {
        if (!runtimeOperationIdMatches(node.operation.id, op.operationId))
            continue;
        total += op.moves.size();
        const auto count = std::min(remaining, op.moves.size());
        remaining -= count;
        if (!count)
            continue;
        const auto bound = toolpath::bindPocketToolParameters(params, op.tool);
        toolpath::DexelToolProfileDefinition profile;
        profile.diameter = bound.tool.diameter;
        profile.cornerRadius = bound.tool.cornerRadius;
        profile.cuttingLength = bound.tool.cuttingEdgeLength;
        gp_Vec axis(setup->zAxis.x, setup->zAxis.y, setup->zAxis.z);
        if (formal->kind == JobOperationKind::Hole)
        {
            const auto &holes = job.holeResult()->machining.operations;
            const auto hole = std::find_if(holes.begin(), holes.end(),
                                           [&](const auto &h) { return h.operationId == op.operationId; });
            if (hole == holes.end())
                throw std::runtime_error("无法绑定钻尖轮廓");
            profile = ipw::holeDexelToolProfile(*hole, params);
            axis = gp_Vec(hole->bottom, hole->entry);
        }
        const toolpath::FixedAxisToolpathView view{op.operationId, axis, op.moves};
        auto applied = stock->applyToolpathPrefix(view, bound, profile, sequence++, count, {});
        if (!applied.stock)
            throw std::runtime_error("无法计算此移动的余料");
        stock = std::move(applied.stock);
    }
    if (!total || remaining)
        throw std::runtime_error("移动序号超出工序范围");
    if (completed == total)
        return job.runtime().ipwCheckpoints.find(node.stateKey)->dexelStock;
    return stock;
}
QJsonArray meshJson(const Handle(Poly_Triangulation) & mesh, const QString &id)
{
    if (mesh.IsNull())
        return {};
    QJsonArray positions, indices;
    for (int i = 1; i <= mesh->NbNodes(); ++i)
    {
        const auto p = mesh->Node(i);
        positions.append(p.X());
        positions.append(p.Y());
        positions.append(p.Z());
    }
    for (int i = 1; i <= mesh->NbTriangles(); ++i)
    {
        const auto &tri = mesh->Triangle(i);
        for (int k = 1; k <= 3; ++k)
            indices.append(tri.Value(k) - 1);
    }

    return {QJsonObject{{"id", id}, {"positions", positions}, {"indices", indices}}};
}

void writeMeshData(const QString &name, const QJsonDocument &document)
{
    QSaveFile file(name);
    const auto bytes = document.toJson(QJsonDocument::Compact);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("无法保存余料网格");
}

QJsonObject compareStock(const MachiningJob &job, Stock stock, const TopoDS_Shape &target,
                         const toolpath::DexelDisplayMeshOptions &opts)
{
    const auto &session = job.ipwSession();
    const auto &checkpoints = job.runtime().ipwCheckpoints;
    const auto root = checkpoints.find(session.initialStateKey());
    auto data = std::make_shared<toolpath::MultiDexelComparisonData>();
    data->initial = root->dexelStock;
    data->final = stock;
    auto before = data->initial;
    const auto &nodes = session.nodes();
    for (const auto &node : nodes)
    {
        auto after = &node == &nodes.back() ? stock : checkpoints.find(node.stateKey)->dexelStock;
        data->operations.push_back({node.operation.id, node.operation.sequence, before, after});
        before = std::move(after);
    }
    toolpath::MultiDexelComparison comparison;
    const auto audit = comparison.compare(data, root->stock, target);
    if (!audit.complete)
        throw std::runtime_error(audit.diagnostics.empty() ? "余料与成品比较失败"
                                                           : audit.diagnostics.front().message);
    const auto display = audit.heightFieldDisplaySource;
    QJsonObject meshes;
    const auto saveView = [&](const QString &key, toolpath::PocketZMapDifferenceView view, double volume) {
        if (volume <= 0 || !display)
            return;
        Handle(Poly_Triangulation) mesh;
        std::string error;
        if (display->buildDisplayMesh(view, mesh, nullptr, {}, &error, {}, opts) && !mesh.IsNull())
            meshes.insert(key, meshJson(mesh, key));
    };
    using View = toolpath::PocketZMapDifferenceView;
    saveView("allowance", View::RemainingAllowance, audit.remainingAllowanceVolume);
    saveView("overcut", View::Overcut, audit.overcutVolume);
    saveView("missing", View::InitialMissing, audit.initialMissingVolume);
    writeMeshData("differences.json", QJsonDocument(meshes));
    QJsonArray attribution;
    if (audit.overcutVolume > std::max(1e-8, 1e-9 * audit.targetVolume))
    {
        const auto attributed = comparison.attributeOvercut(data, target, {}, nullptr, display);
        if (!attributed.complete)
            throw std::runtime_error("工序过切归因失败");
        for (const auto &op : attributed.operations)
            attribution.append(QJsonObject{{"operation", QString::fromStdString(op.operationId)},
                                           {"sequence", op.sequenceIndex},
                                           {"volume", op.volume},
                                           {"certain", op.certainVolume},
                                           {"uncertain", op.boundaryUncertaintyVolume}});
    }
    return {{"allowance", audit.remainingAllowanceVolume},
            {"overcut", audit.overcutVolume},
            {"missing", audit.initialMissingVolume},
            {"certainOvercut", audit.heightFieldCertainOvercutVolume},
            {"boundaryUncertainty", audit.heightFieldBoundaryUncertaintyVolume},
            {"resolution", audit.heightFieldResolution},
            {"attribution", attribution}};
}
} // namespace

QJsonObject stockResult(const MachiningJob &job, const ipw::IpwExecutionResult &simulation,
                        const QJsonObject &request, const TopoDS_Shape &target)
{
    const auto &nodes = job.ipwSession().nodes();
    auto stock = simulation.finalCheckpoint->dexelStock;
    QJsonObject result;
    int completeOperations = static_cast<int>(nodes.size());
    if (request.contains("completedMoves"))
    {
        const int completed = request["completedMoves"].toInt(-1);
        if (completed < 0 || nodes.empty() ||
            nodes.back().operation.id != request["operation"].toString().toStdString())
            throw std::runtime_error("无效的移动仿真请求");
        std::size_t total = 0;
        stock = moveFrame(job, nodes.back(), completed, total);
        result.insert("completedMoves", completed);
        result.insert("totalMoves", static_cast<qint64>(total));
        if (completed < total)
            --completeOperations;
    }
    std::string error;
    Handle(Poly_Triangulation) mesh;
    toolpath::DexelDisplayMeshOptions opts;
    opts.maximumPlanarCellCount = 180000;
    if (!stock->buildDisplayMesh(mesh, nullptr, &error, {}, opts) || mesh.IsNull())
        throw std::runtime_error("IPW mesh: " + error);
    writeMeshData("stock.json", QJsonDocument(meshJson(mesh, "IPW")));
    result.insert("comparison", compareStock(job, stock, target, opts));
    result.insert("volume", stock->volume());
    result.insert("operations", completeOperations);
    const auto initial = job.runtime().ipwCheckpoints.find(job.ipwSession().initialStateKey());
    if (initial)
        result.insert("removedVolume", initial->volume - stock->volume());
    return result;
}

} // namespace oap::worker
