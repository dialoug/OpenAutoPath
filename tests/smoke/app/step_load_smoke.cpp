// 精确索引并行等价性、预算、取消、缓存与预览生命周期回归。
#include "analysis/ModelIO.h"
#include "analysis/StepIndexer.h"
#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "toolpath/core/ToolpathParallel.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QTemporaryDir>
#include <QThread>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomConvert.hxx>
#include <STEPControl_Writer.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <atomic>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
TopoDS_Shape fixture() {
    BRep_Builder builder;
    TopoDS_Compound all;
    builder.MakeCompound(all);
    const auto box = BRepPrimAPI_MakeBox(10,12,14).Shape();
    builder.Add(all,box);
    builder.Add(all,box); // 重复实例必须去重。
    gp_Trsf transform;
    transform.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),0.37);
    transform.SetTranslationPart(gp_Vec(30,5,7));
    builder.Add(all,box.Moved(TopLoc_Location(transform)).Reversed());
    builder.Add(all,BRepPrimAPI_MakeSphere(gp_Pnt(50,0,0),5).Shape()); // 接缝/退化边。
    TopoDS_Shell shell;
    builder.MakeShell(shell);
    for (int patch=0;patch<8;++patch) {
        TColgp_Array2OfPnt poles(1,4,1,4);
        for (int u=1;u<=4;++u) for (int v=1;v<=4;++v)
            poles.SetValue(u,v,gp_Pnt(patch*5+u,v+20,(u==2 && v==2)?2.0:0.0));
        Handle(Geom_BezierSurface) bezier = new Geom_BezierSurface(poles);
        auto surface = GeomConvert::SurfaceToBSplineSurface(bezier);
        builder.Add(shell,BRepBuilderAPI_MakeFace(surface,1e-7).Face());
    }
    builder.Add(all,shell);
    return all;
}
QByteArray serialized(const oap::Model& model, const QString& path) {
    require(oap::writeModel(path.toStdString(),model),"serialize failed");
    QFile file(path);
    require(file.open(QIODevice::ReadOnly),"read serialized model failed");
    return file.readAll();
}
void indexParity(const TopoDS_Shape& shape, const QString& root) {
    auto app=XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF",doc);
    XCAFDoc_DocumentTool::ShapeTool(doc->Main())->AddShape(shape,false);
    oap::StepIndexer reference;
    oap::Model expected;
    require(reference.build(doc,expected),"serial reference failed");
    const auto bytes=serialized(expected,root+"/serial.bin");
    for (std::size_t capacity : {0u,1u,3u}) {
        oap::toolpath::ToolpathParallelBudget budget(capacity);
        oap::toolpath::ToolpathGenerationControl admission;
        admission.tryAcquireParallelWorkers=[&](std::size_t n){return budget.tryAcquire(n);};
        admission.releaseParallelWorkers=[&](std::size_t n){budget.release(n);};
        std::atomic_int active{0},peak{0};
        std::size_t last=0,total=0;
        bool monotonic=true;
        oap::StepIndexBuildControl control;
        control.executeRange=[&](std::size_t count,const std::function<void(std::size_t)>& task){
            oap::toolpath::ToolpathParallelLease lease(&admission,3,3);
            oap::toolpath::toolpathParallelForWorkers(count,lease.parallelism(),[&](std::size_t i){
                const int value=++active;
                int old=peak.load();while(old<value && !peak.compare_exchange_weak(old,value)) {}
                QThread::msleep(2);
                task(i);
                --active;
            });
        };
        control.reportProgress=[&](std::size_t done,std::size_t count){
            monotonic=monotonic && done>=last && done<=count;last=done;total=count;
        };
        oap::StepIndexer parallel;
        oap::Model actual;
        oap::StepIndexBuildStatistics stats;
        require(parallel.build(doc,actual,nullptr,&control,&stats),"parallel index failed");
        require(serialized(actual,root+"/parallel.bin")==bytes,"parallel model differs from serial");
        require(monotonic && last==total && total==actual.faces.size(),"progress must complete monotonically");
        require(stats.isolatedFaceCount==8 && peak<=static_cast<int>(capacity+1),"worker budget not respected");
        require(capacity==0 ? peak==1 : peak>=2,"parallel executor was not exercised");
        require(budget.tryAcquire(capacity)==capacity,"worker lease leaked");
        budget.release(capacity);
        for (const auto& face : actual.faces) {
            const auto* topology=parallel.faceForId(face.face_id);
            require(topology && parallel.idForFace(*topology)==face.face_id,"unstable face ID");
        }
        oap::StepIndexer restored;
        require(restored.restoreMaps(doc,actual),"parallel index cannot restore maps");
    }
    oap::StepIndexer cancelled;
    oap::Model output=expected;
    std::atomic_bool stop{false};
    oap::StepIndexBuildControl control;
    control.executeRange=[](std::size_t count,const std::function<void(std::size_t)>& task){
        oap::toolpath::toolpathParallelForWorkers(count,4,task);
    };
    control.isCancellationRequested=[&]{return stop.load();};
    control.reportProgress=[&](std::size_t done,std::size_t total){if(done+1>=total)stop=true;};
    require(!cancelled.build(doc,output,nullptr,&control),"cancelled build must fail");
    require(output.faces.empty() && !cancelled.faceForId("F1"),"cancelled build published partial state");
    stop=false;
    control.reportProgress={};
    control.executeRange=[](std::size_t count,const std::function<void(std::size_t)>& task){
        oap::toolpath::toolpathParallelForWorkers(count,4,[&](std::size_t i){
            if(i==0)throw std::runtime_error("executor failure");task(i);
        });
    };
    bool caught=false;
    try { cancelled.build(doc,output,nullptr,&control); } catch(const std::runtime_error&) { caught=true; }
    require(caught && output.faces.empty() && !cancelled.faceForId("F1"),"failed executor published partial state");
    app->Close(doc);
}

void waitFor(const bool& done) {
    QElapsedTimer timer;timer.start();
    while(!done && timer.elapsed()<20000){QCoreApplication::processEvents();QThread::msleep(1);}
    require(done,"worker completion timeout");
}
void loadingLifecycle(const QString& file,const QString& cacheRoot) {
    oap::app::ModelDocumentWorkspaceCoordinator workspace(cacheRoot.toStdString());
    bool done=false,installed=false,fromCache=false,isolated=false,gate=true;
    int previews=0;
    std::shared_ptr<AnalysisLoadPreview> captured;
    QObject::connect(&workspace,&oap::app::ModelDocumentWorkspaceCoordinator::previewReady,
                     &workspace,[&](std::shared_ptr<AnalysisLoadPreview> preview){
        ++previews;captured=preview;
        gate=gate && workspace.empty() && workspace.busy() && workspace.acceptsPreview(*preview);
        auto stale=*preview;++stale.requestId;
        gate=gate && !workspace.acceptsPreview(stale);
        stale=*preview;stale.sourcePath+=".old";
        gate=gate && !workspace.acceptsPreview(stale);
    });
    QObject::connect(&workspace,&oap::app::ModelDocumentWorkspaceCoordinator::analysisFinished,
                     &workspace,[&](std::shared_ptr<AnalysisLoadResult> result){
        fromCache=result->fromCache;
        if(captured && result->ok){
            isolated=true;
            for(TopExp_Explorer previewFace(captured->shape,TopAbs_FACE);previewFace.More();previewFace.Next())
                result->stepIndexer->forEachFace([&](const std::string&,const TopoDS_Face& face){
                    isolated=isolated && !face.IsPartner(previewFace.Current()) &&
                        BRep_Tool::Surface(face)!=BRep_Tool::Surface(TopoDS::Face(previewFace.Current()));
                });
        }
        installed=workspace.completeLoad(result).installed();done=true;
    });
    require(workspace.beginLoad(file),"cold load dispatch failed");waitFor(done);
    require(installed && !fromCache && previews==1 && gate && isolated,"preview/atomic install/isolation failed");
    require(!workspace.acceptsPreview(*captured),"completed request accepted a stale preview");
    const auto bytes=serialized(workspace.model(),cacheRoot+"/cold.bin");
    done=false;captured.reset();previews=0;
    require(workspace.beginLoad(file),"warm load dispatch failed");waitFor(done);
    require(installed && fromCache && previews==0,"cache hit must skip preview and exact indexing");
    require(bytes==serialized(workspace.model(),cacheRoot+"/warm.bin"),"cache changed model");
    workspace.cancelAndWait();
}

void cancelPreview(const QString& file,const QString& root) {
    oap::app::ModelDocumentWorkspaceCoordinator workspace(root.toStdString());
    bool done=false,cancelled=false,gate=false;
    QObject::connect(&workspace,&oap::app::ModelDocumentWorkspaceCoordinator::previewReady,
                     &workspace,[&](std::shared_ptr<AnalysisLoadPreview> preview){
        workspace.requestCancelLoad();gate=!workspace.acceptsPreview(*preview);
    });
    QObject::connect(&workspace,&oap::app::ModelDocumentWorkspaceCoordinator::analysisFinished,
                     &workspace,[&](std::shared_ptr<AnalysisLoadResult> result){
        cancelled=workspace.completeLoad(result).status==oap::app::DocumentLoadStatus::Cancelled;done=true;
    });
    require(workspace.beginLoad(file),"cancel load dispatch failed");waitFor(done);
    require(gate && cancelled && workspace.empty() && !workspace.busy(),"cancelled preview installed a document");
    workspace.cancelAndWait();
}
}
int main(int argc,char** argv){
    QCoreApplication app(argc,argv);
    try {
        QTemporaryDir temp;require(temp.isValid(),"temporary directory failed");
        const auto shape=fixture();
        indexParity(shape,temp.path());
        const QString path=temp.filePath("compound.step");
        STEPControl_Writer writer;
        require(writer.Transfer(shape,STEPControl_AsIs)==IFSelect_RetDone,"STEP transfer failed");
        require(writer.Write(path.toStdString().c_str())==IFSelect_RetDone,"STEP write failed");
        loadingLifecycle(path,temp.filePath("cache-test"));
        cancelPreview(path,temp.filePath("cancel-test"));
        std::cout<<"step load parity, budgets, cancellation, preview and cache passed\n";
        return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
