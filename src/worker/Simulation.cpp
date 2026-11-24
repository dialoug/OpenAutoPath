#include "support/Files.h"
#include <bit>
#include "support/Json.h"
#include <vector>
#include <string>
#include <cstdint>
#include "Simulation.h"
#include "toolpath/pocket/simulation/IpwQuality.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/simulation/FixedAxisToolpathView.h"
#include <algorithm>
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include <map>
#include <array>
#include "toolpath/core/PocketToolBinding.h"
#include <stdexcept>

namespace oap::worker
{
using namespace manufacturing;
namespace
{
using Stock = std::shared_ptr<const toolpath::MultiDexelStock>;

struct FamilyProgram
{
    toolpath::ToolpathProgram program;
    toolpath::ToolpathParams params;
};

FamilyProgram familyProgram(const MachiningJob &job, ipw::IpwOperationKind kind)
{
    switch (kind) {
    case ipw::IpwOperationKind::Hole:
        return {job.holeResult()->toProgram(), job.holeResult()->effectiveParams};
    case ipw::IpwOperationKind::Pocket:
        return {job.pocketResult()->toProgram(), job.pocketResult()->effectiveParams};
    case ipw::IpwOperationKind::Profile:
        return {job.profileResult()->toProgram(), job.profileResult()->effectiveParams};
    case ipw::IpwOperationKind::SurfaceFinishing:
        return {job.currentFinishingResult()->toProgram(), job.currentFinishingResult()->effectiveParams};
    default:
        throw std::runtime_error("未知余料工序");
    }
}

struct Cut
{
    toolpath::FixedAxisToolpathView path;
    toolpath::ToolpathParams params;
    toolpath::DexelToolProfileDefinition profile;
    toolpath::StockSimulationOptions options;
};

std::vector<Cut> operationCuts(const MachiningJob &job, const ipw::IpwNode &node, const FamilyProgram &family)
{
    const auto *setup = job.setupById(node.operation.setupId);
    if (!setup)
        throw std::runtime_error("缺少工序装夹");
    std::vector<Cut> cuts;
    for (const auto &op : family.program.operations) {
        if (!runtimeOperationIdMatches(node.operation.id, op.operationId) || op.moves.empty())
            continue;
        auto params = toolpath::bindPocketToolParameters(family.params, op.tool);
        toolpath::DexelToolProfileDefinition profile;
        profile.diameter = params.tool.diameter;
        profile.cornerRadius = params.tool.cornerRadius;
        profile.cuttingLength = params.tool.cuttingEdgeLength;
        gp_Vec axis(setup->zAxis.x, setup->zAxis.y, setup->zAxis.z);
        if (node.operation.kind == ipw::IpwOperationKind::Hole) {
            const auto &holes = job.holeResult()->machining.operations;
            const auto hole = std::find_if(holes.begin(), holes.end(),
                [&](const auto &h) { return h.operationId == op.operationId; });
            if (hole == holes.end())
                throw std::runtime_error("无法绑定钻尖轮廓");
            profile = ipw::holeDexelToolProfile(*hole, family.params);
            axis = gp_Vec(hole->bottom, hole->entry);
        }
        auto options = toolpath::StockSimulationOptions{};
        if (node.operation.kind == ipw::IpwOperationKind::SurfaceFinishing)
            options = ipw::engravingSweepOptions(options, op.moves.size());
        cuts.push_back({{op.operationId, axis, op.moves}, std::move(params), profile, options});
    }
    if (cuts.empty())
        throw std::runtime_error("工序缺少可回放的刀路");
    return cuts;
}

Stock checkpoint(const MachiningJob &job, const std::string &key)
{
    const auto saved = job.runtime().ipwCheckpoints.find(key);
    if (!saved || !saved->dexelStock)
        throw std::runtime_error("缺少工序余料检查点");
    return saved->dexelStock;
}

// Apply only the moves between two successive samples using the same cutter
// definition as the exact, single-step inspection.
Stock advanceStock(Stock stock, const Cut &cut, std::size_t start, std::size_t count, int sequence)
{
    auto range = cut.path;
    range.moves = range.moves.subspan(start, count);
    const auto result = stock->applyToolpathPrefix(range, cut.params, cut.profile, sequence, count,
        cut.options);
    if (!result.stock)
        throw std::runtime_error("无法计算此移动的余料");
    return result.stock;
}

Stock moveFrame(const MachiningJob &job, const ipw::IpwNode &node, std::size_t completed, std::size_t &total)
{
    const auto family = familyProgram(job, node.operation.kind);
    const auto cuts = operationCuts(job, node, family);
    for (const auto &cut : cuts)
        total += cut.path.moves.size();
    if (completed > total)
        throw std::runtime_error("移动序号超出工序范围");
    if (completed == total)
        return checkpoint(job, node.stateKey);
    auto stock = checkpoint(job, node.parentStateKey);
    for (std::size_t i = 0; i < cuts.size() && completed; ++i) {
        const auto count = std::min(completed, cuts[i].path.moves.size());
        stock = advanceStock(stock, cuts[i], 0, count, static_cast<int>(i));
        completed -= count;
    }
    return stock;
}

oap::JsonArray meshJson(const Handle(Poly_Triangulation) & mesh, const std::string &id)
{
    if (mesh.IsNull())
        return {};
    oap::JsonArray positions, indices;
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

    return {oap::JsonObject{{"id", id}, {"positions", positions}, {"indices", indices}}};
}

void writeMeshData(const std::string &name, const oap::JsonDocument &document)
{
    const auto bytes = document.toJson(oap::JsonDocument::Compact);
    if (!oap::writeFileAtomically(name, bytes))
        throw std::runtime_error("无法保存余料网格");
}

oap::JsonObject compareStock(const MachiningJob &job, Stock stock, const TopoDS_Shape &target,
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
    oap::JsonObject meshes;
    const auto saveView = [&](const std::string &key, toolpath::PocketZMapDifferenceView view, double volume) {
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
    writeMeshData("differences.json", oap::JsonDocument(meshes));
    oap::JsonArray attribution;
    if (audit.overcutVolume > std::max(1e-8, 1e-9 * audit.targetVolume))
    {
        const auto attributed = comparison.attributeOvercut(data, target, {}, nullptr, display);
        if (!attributed.complete)
            throw std::runtime_error("工序过切归因失败");
        for (const auto &op : attributed.operations)
            attribution.append(oap::JsonObject{{"operation", op.operationId},
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

oap::JsonObject replayResult(const MachiningJob &job, bool selectedOnly, int quality)
{
    const auto &nodes = job.ipwSession().nodes();
    if (nodes.empty())
        throw std::runtime_error("没有可回放的工序");
    // Keep each family's move storage alive for every non-owning Cut view.
    std::map<ipw::IpwOperationKind, FamilyProgram> families;
    struct Operation {
        const ipw::IpwNode &node;
        std::vector<Cut> cuts;
    };
    std::vector<Operation> operations;
    oap::JsonArray path, spans, frames, tools;
    const auto point = [](const gp_Pnt &p) { return oap::JsonArray{p.X(), p.Y(), p.Z()}; };
    for (const auto &node : nodes) {
        if (selectedOnly && &node != &nodes.back())
            continue;
        const auto kind = node.operation.kind;
        if (!families.contains(kind))
            families.emplace(kind, familyProgram(job, kind));
        auto cuts = operationCuts(job, node, families.at(kind));
        for (const auto &cut : cuts) {
            for (const auto &move : cut.path.moves)
                path.append(oap::JsonArray{point(move.from), point(move.to)});
            toolpath::DexelToolProfile profile;
            oap::JsonArray outline;
            if (toolpath::buildDexelToolProfile(cut.profile, {}, profile)) {
                for (const auto &band : profile.bands) {
                    outline.append(oap::JsonArray{band.radius, band.lower});
                    outline.append(oap::JsonArray{band.radius, band.upper});
                }
            }
            const auto &axis = cut.path.outwardToolAxis;
            tools.append(oap::JsonObject{{"end", path.size()}, {"axis", oap::JsonArray{axis.X(), axis.Y(), axis.Z()}},
                                     {"outline", outline}});
        }
        spans.append(oap::JsonObject{{"id", node.operation.id}, {"end", path.size()}});
        operations.push_back({node, std::move(cuts)});
    }
    writeMeshData("path.json", oap::JsonDocument(path));
    oap::AtomicFile file("replay.bin");
    if (!file.stream()) throw std::runtime_error("无法写入余料回放");
    auto writeWord = [&](std::uint32_t word) {
        const std::array bytes{
            static_cast<char>(word), static_cast<char>(word >> 8),
            static_cast<char>(word >> 16), static_cast<char>(word >> 24)};
        file.stream().write(bytes.data(), bytes.size());
    };
    auto saveFrame = [&](const Stock &stock, std::size_t move) {
        const auto opts = toolpath::ipwDisplayQualityProfile(quality, stock->resolution()).meshOptions;
        toolpath::DexelDisplayMeshMetrics metrics;
        Handle(Poly_Triangulation) mesh;
        std::string error;
        if (!stock->buildDisplayMesh(mesh, &metrics, &error, {}, opts) || mesh.IsNull())
            throw std::runtime_error("余料回放网格: " + error);
        // The native surface duplicates the vertices of every triangle. Index
        // identical transmitted positions, keeping the original facets and winding.
        std::map<std::array<float, 3>, std::uint32_t> vertices;
        std::vector<std::uint32_t> indices(mesh->NbNodes() + 1);
        for (int i = 1; i <= mesh->NbNodes(); ++i) {
            const auto p = mesh->Node(i);
            const std::array<float, 3> xyz{float(p.X()), float(p.Y()), float(p.Z())};
            const auto [it, inserted] = vertices.try_emplace(xyz, static_cast<std::uint32_t>(vertices.size()));
            if (inserted)
                for (const float coordinate : xyz) writeWord(std::bit_cast<std::uint32_t>(coordinate));
            indices[i] = it->second;
        }
        const auto triangles = mesh->NbTriangles();
        for (int i = 1; i <= triangles; ++i) {
            const auto &t = mesh->Triangle(i);
            for (int corner = 1; corner <= 3; ++corner) writeWord(indices[t.Value(corner)]);
        }
        frames.append(oap::JsonObject{{"move", move}, {"vertices", static_cast<std::int64_t>(vertices.size())}, {"triangles", triangles},
                                  {"volume", stock->volume()}, {"materialSpacing", stock->resolution()},
                                  {"displaySpacing", stock->resolution() * metrics.planarDecimationStride}});
    };
    // At most 33 display meshes per replay, independent of program length.
    const auto stride = (path.size() + 31) / 32;
    std::size_t cursor = 0;
    saveFrame(checkpoint(job, operations.front().node.parentStateKey), cursor);
    for (const auto &[node, cuts] : operations) {
        auto stock = checkpoint(job, node.parentStateKey);
        for (std::size_t i = 0; i < cuts.size(); ++i) {
            const auto &cut = cuts[i];
            for (std::size_t start = 0; start < cut.path.moves.size();) {
                const auto count = std::min<std::size_t>(stride - cursor % stride, cut.path.moves.size() - start);
                const bool endOfOperation = i + 1 == cuts.size() && start + count == cut.path.moves.size();
                stock = endOfOperation ? checkpoint(job, node.stateKey)
                                       : advanceStock(stock, cut, start, count, static_cast<int>(i));
                cursor += count;
                start += count;
                if (cursor % stride == 0 || cursor == path.size())
                    saveFrame(stock, cursor);
            }
        }
    }
    if (!file.commit())
        throw std::runtime_error("无法保存余料回放");
    return {{"frames", frames}, {"operations", spans}, {"tools", tools}, {"moves", path.size()},
            {"quality", quality}, {"selectedOnly", selectedOnly}};
}

oap::JsonObject stockResult(const MachiningJob &job, const ipw::IpwExecutionResult &simulation,
                        const oap::JsonObject &request, const TopoDS_Shape &target)
{
    const auto &nodes = job.ipwSession().nodes();
    auto stock = simulation.finalCheckpoint->dexelStock;
    oap::JsonObject result;
    int completeOperations = static_cast<int>(nodes.size());
    if (request.contains("completedMoves"))
    {
        const int completed = request["completedMoves"].toInt(-1);
        if (completed < 0 || nodes.empty() ||
            nodes.back().operation.id != request["operation"].toString())
            throw std::runtime_error("无效的移动仿真请求");
        std::size_t total = 0;
        stock = moveFrame(job, nodes.back(), completed, total);
        result.insert("completedMoves", completed);
        result.insert("totalMoves", static_cast<std::int64_t>(total));
        if (completed < total)
            --completeOperations;
    }
    std::string error;
    Handle(Poly_Triangulation) mesh;
    const int quality = request["quality"].toInt(5);
    const auto opts = toolpath::ipwDisplayQualityProfile(quality, stock->resolution()).meshOptions;
    toolpath::DexelDisplayMeshMetrics metrics;
    if (!stock->buildDisplayMesh(mesh, &metrics, &error, {}, opts) || mesh.IsNull())
        throw std::runtime_error("IPW mesh: " + error);
    writeMeshData("stock.json", oap::JsonDocument(meshJson(mesh, "IPW")));
    result.insert("comparison", compareStock(job, stock, target, opts));
    result.insert("volume", stock->volume());
    result.insert("quality", quality);
    result.insert("materialSpacing", stock->resolution());
    result.insert("displaySpacing", metrics.planarDecimationStride * stock->resolution());
    result.insert("operations", completeOperations);
    const auto initial = job.runtime().ipwCheckpoints.find(job.ipwSession().initialStateKey());
    if (initial)
        result.insert("removedVolume", initial->volume - stock->volume());
    return result;
}

} // namespace oap::worker
