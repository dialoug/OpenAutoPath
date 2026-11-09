#include "toolpath/profile/planning/ProfileDirectionPlanner.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"
#include "tooling/library/ToolLibraryStorage.h"
#include "toolpath/core/ToolpathParallel.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

namespace {
using namespace oap::toolpath;
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

TopoDS_Shape firstSolid(const TopoDS_Shape& shape) { TopExp_Explorer solids(shape, TopAbs_SOLID); return solids.More() ? solids.Current() : TopoDS_Shape{}; }

std::vector<ProfileRegion> boxRegions(const TopoDS_Shape& target)
{
    std::vector<ProfileRegion> result;
    const std::array<gp_Dir, 6> directions{{{0,0,1}, {1,0,0}, {0,1,0}, {-1,0,0}, {0,-1,0}, {0,0,-1}}};
    for (const auto& direction : directions) {
        ProfileRegion region;
        region.source.unitId = "unit";
        region.source.rootFaceId = "root-" + std::to_string(result.size());
        region.openDir = direction;
        region.frame = gp_Ax3(gp_Pnt(), direction);
        double u0=1.e9,v0=1.e9,w0=1.e9,u1=-1.e9,v1=-1.e9,w1=-1.e9;
        for (double x : {0.,120.}) for(double y : {0.,80.}) for(double z : {0.,20.}) {
            const gp_Vec point(x,y,z);
            const double u=point.Dot(gp_Vec(region.frame.XDirection()));
            const double v=point.Dot(gp_Vec(region.frame.YDirection()));
            const double w=point.Dot(gp_Vec(direction));
            u0=std::min(u0,u);u1=std::max(u1,u);v0=std::min(v0,v);v1=std::max(v1,v);w0=std::min(w0,w);w1=std::max(w1,w);
        }
        region.frame.SetLocation(gp_Pnt().Translated(gp_Vec(direction)*w0));
        region.depth=w1-w0;
        region.boundary.points={{u0,v0},{u1,v0},{u1,v1},{u0,v1}};
        region.boundary.closed=true;
        region.boundary.signedArea=(u1-u0)*(v1-v0);
        region.protectedMaterialShape=firstSolid(target);
        BRep_Builder builder;builder.MakeCompound(region.sectionShape);
        for(TopExp_Explorer faces(target,TopAbs_FACE);faces.More();faces.Next())builder.Add(region.sectionShape,faces.Current());
        result.push_back(std::move(region));
    }
    return result;
}

void checkSynthetic()
{
    const auto box=BRepPrimAPI_MakeBox(120,80,20).Shape();
    const auto pocket=BRepAlgoAPI_Cut(box,BRepPrimAPI_MakeBox(gp_Pnt(10,10,12),30,30,10).Shape()).Shape();
    const auto regions=boxRegions(pocket);
    ProfileDirectionPlanningOptions options;
    options.hasCandidateTool=[](const auto& r){return r.depth<=110;};
    auto plan=ProfileDirectionPlanner().plan(regions,options);
    require(plan.complete && plan.selectedRegionIndices==std::vector<std::size_t>{0} && plan.coverage.front().coveredRegions.size()==6,
        "top-open pocket must retain only the proven +Z external envelope");
    options.hasCandidateTool=[](const auto&){return false;};
    require(ProfileDirectionPlanner().plan(regions,options).selectedRegionIndices.size()==6,"unreachable candidates silently dropped tasks");
    options.hasCandidateTool=[](const auto& r){return r.depth<=30;};
    const auto sideHole=BRepAlgoAPI_Cut(pocket,BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-1,60,10),gp_Dir(1,0,0)),3,122).Shape()).Shape();
    require(ProfileDirectionPlanner().plan(boxRegions(sideHole),options).selectedRegionIndices.size()==6,
        "a side opening must not be discarded by choosing the thin direction");
    const auto step=BRepAlgoAPI_Cut(box,BRepPrimAPI_MakeBox(gp_Pnt(-1,0,10),30,81,11).Shape()).Shape();
    require(ProfileDirectionPlanner().plan(boxRegions(step),options).selectedRegionIndices.size()==6,
        "an external step must not be treated as a constant envelope");
    options.stockMargins=gp_Vec(1,1,1);
    require(ProfileDirectionPlanner().plan(regions,options).selectedRegionIndices.size()==6,"end stock allowance was silently discarded");
    options.stockMargins=gp_Vec(0,0,0);options.allowReduction=false;
    require(ProfileDirectionPlanner().plan(regions,options).selectedRegionIndices.size()==6,"unsupported stock/depth scope was reduced");
    options.allowReduction=true;
    auto withInternalRoot=regions;
    auto internal=regions.front();internal.depth=12;internal.boundary.points={{12,12},{20,12},{20,20},{12,20}};
    withInternalRoot.push_back(internal);
    require(ProfileDirectionPlanner().plan(withInternalRoot,options).selectedRegionIndices.size()==7,"an internal root was mistaken for a duplicate external task");
    auto rotated=regions;
    gp_Trsf transform;transform.SetRotation(gp_Ax1(gp_Pnt(),gp_Dir(1,2,3)),0.41);
    const auto rotatedShape=BRepBuilderAPI_Transform(pocket,transform,true).Shape();
    for(auto& r:rotated){r.frame.Transform(transform);r.openDir.Transform(transform);r.protectedMaterialShape=firstSolid(rotatedShape);}
    require(ProfileDirectionPlanner().plan(rotated,options).selectedRegionIndices.size()==6,"oblique world-box stock end wedges were ignored");
    auto units=regions;
    gp_Trsf translation;translation.SetTranslation(gp_Vec(200,0,0));
    const auto second=BRepBuilderAPI_Transform(pocket,translation,true).Shape();
    for(auto r:regions){r.frame.Transform(translation);r.protectedMaterialShape=firstSolid(second);units.push_back(r);}
    require(ProfileDirectionPlanner().plan(units,options).selectedRegionIndices==std::vector<std::size_t>({0,6}),"different solids were merged into one task");
    int polls=0;options.cancellationRequested=[&]{return ++polls>4;};
    plan=ProfileDirectionPlanner().plan(regions,options);
    require(plan.cancelled && !plan.complete && plan.selectedRegionIndices.empty() && plan.coverage.empty(),"cancelled planning published a partial selection");
}

void checkStep(const char* path, const char* libraryPath)
{
    STEPControl_Reader reader;
    require(reader.ReadFile(path)==IFSelect_RetDone && reader.TransferRoots()>0,"STEP read failed");
    Handle(TDocStd_Document) document;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF",document);
    XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(reader.OneShape(),Standard_False);
    oap::Model model;oap::StepIndexer indexer;
    require(indexer.build(document,model),"STEP index failed");
    oap::ModelIndex modelIndex;modelIndex.build(model);
    oap::ThreeAxisWireTraceRequest request;request.model=&model;request.modelIndex=&modelIndex;request.stepIndexer=&indexer;
    const auto trace=oap::ThreeAxisWireTraceUseCase().execute(request);
    require(trace.ok(),"STEP recognition failed");
    std::vector<RootOuterWireProfileSource> sources;
    for(const auto& source:trace.profileSources){
        RootOuterWireProfileSource mapped;
        mapped.unitId=source.unitId;mapped.axisLabel=source.axisLabel;mapped.normalDirection=source.normalDirection;
        mapped.rootFaceId=source.rootFaceId;mapped.rootWireId=source.rootWireId;mapped.sourceFaceIds=source.sourceFaceIds;
        mapped.machiningFaceIds=source.machiningFaceIds;mapped.terminalFaceIds=source.terminalFaceIds;mapped.mainOuterFaceIds=source.mainOuterFaceIds;
        sources.push_back(std::move(mapped));
    }
    auto regions=ProfileRegionBuilder().build(sources,indexer);
    std::stable_sort(regions.begin(),regions.end(),[](const auto& a,const auto& b){return a.openDir.Z()>b.openDir.Z();});
    oap::tooling::ToolLibrary library;
    ToolpathParams params;
    QString libraryError;
    if(libraryPath)require(oap::tooling::ToolLibraryStorage::loadLibrary(QString::fromUtf8(libraryPath),library,&libraryError),"test tool library load failed");
    ProfileDirectionPlanningOptions options;
    options.hasCandidateTool=[&](const auto& r){return libraryPath ? oap::tooling::ProfileAutomaticToolpath::hasCandidateTool(library,r,params) : r.depth<=110;};
    const auto plan=ProfileDirectionPlanner().plan(regions,options);
    std::cout<<"STEP regions="<<regions.size()<<" selected="<<plan.selectedRegionIndices.size()<<'\n';
    for(const auto i:plan.selectedRegionIndices)std::cout<<"root="<<regions[i].source.rootFaceId<<" axis="<<regions[i].openDir.X()<<','<<regions[i].openDir.Y()<<','<<regions[i].openDir.Z()<<" depth="<<regions[i].depth<<'\n';
    require(regions.size()>1 && plan.complete && plan.selectedRegionIndices.size()==1 && regions[plan.selectedRegionIndices.front()].openDir.Z()>0.99,
        "cam feature STEP was not reduced to its proven +Z envelope");
    if(libraryPath) {
        std::vector<ProfileRegion> selected;
        for(const auto i:plan.selectedRegionIndices)selected.push_back(regions[i]);
        for(auto& region:selected)ProfileRegionBuilder::updateStockBoundary(region);
        ToolpathGenerationControl control;attachGlobalToolpathParallelBudget(control);
        const auto generated=oap::tooling::ProfileAutomaticToolpath().generate(library,selected,params,"DIRECTION_TEST_",&control);
        std::cout<<"generated="<<generated.ok()<<" operations="<<generated.batch.operations.size()<<" error="<<generated.error<<'\n';
        require(generated.ok() && generated.batch.failures.empty() && generated.batch.safetyValidationPerformed,
            "selected STEP direction failed full automatic tool/path validation");
        for(const auto& operation:generated.batch.operations)
            std::cout<<"tool="<<operation.tool.toolNumber<<" diameter="<<operation.tool.diameter<<" moves="<<operation.contour.moves.size()<<'\n';
    }
    XCAFApp_Application::GetApplication()->Close(document);
}
}

int main(int argc,char** argv)
{
    try {checkSynthetic();if(argc>1)checkStep(argv[1],argc>2?argv[2]:nullptr);std::cout<<"Profile direction coverage checks passed\n";return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
