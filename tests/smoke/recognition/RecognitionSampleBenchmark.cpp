#include "RecognitionSampleBenchmark.h"

#include "analysis/ModelIO.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "analysis/Utf8Path.h"
#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "toolpath/core/ToolpathParallel.h"

#include <BinXCAFDrivers.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <XCAFApp_Application.hxx>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
// Length-prefixed strings avoid delimiter collisions in topology identifiers.
struct Record {
    std::ostringstream out;
    Record() { out.imbue(std::locale::classic()); out << std::setprecision(17); }
    Record& add(const std::string& value) {
        out << value.size() << ':' << value << ';'; return *this;
    }
    template<class T> Record& add(T value) { out << value << ';'; return *this; }
    template<class Range> Record& faces(const Range& values) {
        std::vector<std::string> sorted(values.begin(), values.end());
        std::sort(sorted.begin(), sorted.end());
        add(sorted.size());
        for (const auto& value : sorted) add(value);
        return *this;
    }
    std::string str() const { return out.str(); }
};

void records(std::ostringstream& output, const char* name, std::vector<std::string> values)
{
    std::sort(values.begin(), values.end());
    output << name << ' ' << values.size() << '\n';
    for (const auto& value : values) output << value << '\n';
}

std::string recognitionSignature(const oap::RecognitionResult& result)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "recognition.counts " << Record().add(result.holeCount)
        .add(result.slotCount).add(result.pocketCount).str() << '\n';
    std::vector<std::string> values;
    for (const auto& instance : result.instances)
        values.push_back(Record().add(static_cast<int>(instance.kind)).add(instance.manual)
            .add(instance.floorFaceId).faces(instance.faceIds).str());
    records(output, "recognition.instances", std::move(values));
    values.clear();
    for (const auto& [face, kind] : result.faceKind)
        values.push_back(Record().add(face).add(static_cast<int>(kind)).str());
    records(output, "recognition.faceKind", std::move(values));
    values.clear();
    for (const auto& hole : result.holeGeometries) {
        values.push_back(Record().add(static_cast<int>(hole.kind))
            .add(hole.entry.X()).add(hole.entry.Y()).add(hole.entry.Z())
            .add(hole.bottom.X()).add(hole.bottom.Y()).add(hole.bottom.Z())
            .add(hole.radius).add(static_cast<int>(hole.bottomProfile))
            .faces(hole.sourceFaceIds).faces(hole.stageWallFaceIds)
            .add(static_cast<int>(hole.stageRole)).add(static_cast<int>(hole.parentRelation))
            .add(hole.parentCounterboreFaceId).add(hole.parentCounterboreFloorFaceId)
            .add(hole.parentCounterboreRadius).str());
    }
    records(output, "recognition.holeGeometries", std::move(values));
    return output.str();
}
} // namespace

std::string completeThreeAxisRecognitionSignature(const oap::ThreeAxisWireTraceUseCaseResult& result)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "threeAxis.combinedTrace " << Record().faces(result.combinedTraceFaceIds).str() << '\n';
    output << "threeAxis.throughStructure " << Record().faces(result.throughStructureFaceIds).str() << '\n';
    output << "threeAxis.axisParallelPlane " << Record().faces(result.axisParallelPlaneFaceIds).str() << '\n';
    std::vector<std::string> values;
    for (const auto& stats : result.axisStats) {
        values.push_back(Record().add(stats.axisLabel).add(stats.rootFaceCount)
            .add(stats.activePathCount).add(stats.blindSlotPathCount).add(stats.blindPocketPathCount)
            .add(stats.terminalFaceCount).add(stats.skippedInvalidFaceCount)
            .faces(stats.planeFaceIds).faces(stats.traceFaceIds).str());
    }
    records(output, "threeAxis.axisStats", std::move(values));
    values.clear();
    for (const auto& subset : result.featureSubsets) {
        values.push_back(Record().add(subset.axisLabel).add(subset.rootFaceId).add(subset.rootWireId)
            .add(subset.fromInnerWire).add(subset.rootIsMainOuterFace)
            .add(static_cast<int>(subset.featureKind)).faces(subset.faceIds)
            .add(subset.rootCanConnectBlindPockets).faces(subset.terminalFaceIds)
            .add(subset.mergedSourceCount).str());
    }
    records(output, "threeAxis.featureSubsets", std::move(values));
    values.clear();
    for (const auto& source : result.pocketSources) {
        values.push_back(Record().add(static_cast<int>(source.kind)).faces(source.sourceFaceIds)
            .add(source.rootFaceId).add(source.rootWireId).add(source.allowOpenSectionContours)
            .add(source.openBoundaryFaceCount).str());
    }
    records(output, "threeAxis.pocketSources", std::move(values));
    values.clear();
    for (const auto& source : result.profileSources) {
        values.push_back(Record().add(source.unitId).add(source.axisLabel).add(source.normalDirection)
            .add(source.rootFaceId).add(source.rootWireId).faces(source.sourceFaceIds)
            .faces(source.machiningFaceIds).faces(source.terminalFaceIds)
            .faces(source.mainOuterFaceIds).str());
    }
    records(output, "threeAxis.profileSources", std::move(values));
    output << "threeAxis.summary " << Record().add(result.failedUnitCount).add(result.verticalSurfaceCount)
        .add(result.excludedHoleFaceCount).add(result.excludedMainOuterFaceCount)
        .add(result.rejectedWithoutNonZNeighborCount).add(result.rejectedNonConvexCount)
        .add(result.overwrittenFaceCount).add(result.absorbedBlindPocketSubsetCount)
        .add(result.cancelled).add(result.error).str() << '\n';
    return output.str();
}

int runRecognitionSampleBenchmark(int argc, char** argv)
{
    Handle(XCAFApp_Application) application;
    Handle(TDocStd_Document) document;
    const auto closeDocument = [&] {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
            document.Nullify();
        }
    };
    try {
        std::string cacheDirectory, snapshotPath;
        bool skipHoles = false;
        for (int index = 1; index < argc; ++index) {
            const std::string_view argument(argv[index]);
            if (argument == "--skip-holes") { skipHoles = true; continue; }
            if ((argument == "--cached-model" || argument == "--snapshot") && index + 1 < argc) {
                (argument == "--cached-model" ? cacheDirectory : snapshotPath) = argv[++index];
                continue;
            }
            throw std::runtime_error("Usage: --cached-model <dir> --snapshot <path> [--skip-holes]");
        }
        if (cacheDirectory.empty() || snapshotPath.empty())
            throw std::runtime_error("Both --cached-model and --snapshot are required.");
        const auto directory = oap::utf8Path(cacheDirectory);
        application = XCAFApp_Application::GetApplication();
        BinXCAFDrivers::DefineFormat(application);
        const auto geometry = oap::pathUtf8(directory / "geometry.xbf");
        if (application->Open(TCollection_ExtendedString(geometry.c_str(), Standard_True), document) != PCDM_RS_OK || document.IsNull())
            throw std::runtime_error("Cannot load cached geometry.xbf.");
        oap::Model model;
        if (!oap::readModel(oap::pathUtf8(directory / "model.bin"), model))
            throw std::runtime_error("Cannot load cached model.bin.");
        oap::StepIndexer indexer;
        if (!indexer.restoreMaps(document, model))
            throw std::runtime_error("Cannot restore cached topology mappings.");
        oap::ModelIndex modelIndex;
        modelIndex.build(model);
        std::cout << "sample units=" << model.units.size() << " faces=" << model.faces.size()
            << " hole_recognition=" << (skipHoles ? "SKIPPED (explicit --skip-holes)" : "enabled") << std::endl;
        using Clock = std::chrono::steady_clock;
        const auto started = Clock::now();
        oap::RecognitionResult recognition;
        if (!skipHoles) recognition = oap::FeatureRecognizer().recognize(model, indexer);
        const auto holesFinished = Clock::now();
        oap::ThreeAxisWireTraceRequest request;
        request.model = &model;
        request.modelIndex = &modelIndex;
        request.stepIndexer = &indexer;
        for (const auto& [face, kind] : recognition.faceKind) {
            if (kind == oap::FeatureKind::ThroughHole || kind == oap::FeatureKind::BlindHole || kind == oap::FeatureKind::CountersinkHole)
                request.recognizedHoleFaceIds.insert(face);
        }
        oap::ThreeAxisWireTraceControl control;
        control.executeRange = [](std::size_t count, const auto& task) {
            oap::toolpath::toolpathParallelForBudgeted(count, task);
        };
        const auto traceStarted = Clock::now();
        const auto trace = oap::ThreeAxisWireTraceUseCase().execute(request, &control);
        const auto finished = Clock::now();
        std::ofstream snapshot(oap::utf8Path(snapshotPath), std::ios::binary | std::ios::trunc);
        if (!snapshot) throw std::runtime_error("Cannot open snapshot output.");
        snapshot << "recognition-snapshot-v1\n" << recognitionSignature(recognition)
            << completeThreeAxisRecognitionSignature(trace);
        snapshot.close();
        if (!snapshot) throw std::runtime_error("Cannot write complete snapshot output.");
        std::cout << "hole_ms=" << std::chrono::duration<double, std::milli>(holesFinished - started).count()
            << " three_axis_ms=" << std::chrono::duration<double, std::milli>(finished - traceStarted).count()
            << " subsets=" << trace.featureSubsets.size() << " pockets=" << trace.pocketSources.size()
            << " profiles=" << trace.profileSources.size() << " failed_units=" << trace.failedUnitCount
            << " snapshot=" << snapshotPath << std::endl;
        closeDocument();
        if (!trace.ok() || trace.failedUnitCount != 0) {
            std::cerr << "Incomplete recognition: " << trace.error << '\n';
            return 1;
        }
        return 0;
    } catch (const Standard_Failure& error) {
        std::cerr << "Recognition benchmark OCCT error: " << error.GetMessageString() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Recognition benchmark error: " << error.what() << '\n';
    }
    try { closeDocument(); } catch (...) {}
    return 1;
}
