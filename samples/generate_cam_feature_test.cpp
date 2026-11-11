#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Ax2.hxx>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <numbers>
#include <iomanip>
TopoDS_Shape box(double x,double y,double z,double dx,double dy,double dz) {return BRepPrimAPI_MakeBox(gp_Pnt(x,y,z),dx,dy,dz).Shape();}
TopoDS_Shape cylinder(double x,double y,double z,double r,double h) {return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),r,h).Shape();}
TopoDS_Shape clean(const TopoDS_Shape& shape) {ShapeUpgrade_UnifySameDomain u(shape,true,true,true);u.Build();return u.Shape();}
TopoDS_Shape fuse(const TopoDS_Shape& a,const TopoDS_Shape& b) {BRepAlgoAPI_Fuse f(a,b);if(!f.IsDone())throw std::runtime_error("fuse failed");return clean(f.Shape());}
void cut(TopoDS_Shape& a,const TopoDS_Shape& b) {BRepAlgoAPI_Cut c(a,b);if(!c.IsDone())throw std::runtime_error("cut failed");a=clean(c.Shape());if(!BRepCheck_Analyzer(a).IsValid())throw std::runtime_error("invalid cut");}
double volume(const TopoDS_Shape& s) {GProp_GProps p;BRepGProp::VolumeProperties(s,p);return p.Mass();}
int main(int argc,char** argv) {
 try {
 if(argc!=2)throw std::runtime_error("output STEP path required");
 BRepBuilderAPI_MakePolygon outline;
 for(const auto& p:{gp_Pnt(0,0,0),gp_Pnt(100,0,0),gp_Pnt(100,10,0),gp_Pnt(120,10,0),gp_Pnt(120,65,0),gp_Pnt(105,80,0),gp_Pnt(0,80,0)})outline.Add(p);
 outline.Close();TopoDS_Shape part=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(outline.Wire()).Face(),gp_Vec(0,0,20)).Shape();
 // Rounded rectangular pocket: X15..60, Y25..60, R4, floor Z12.
 auto pocket=fuse(box(19,25,12,37,35,9),box(15,29,12,45,27,9));
 for(double x:{19.,56.})for(double y:{29.,56.})pocket=fuse(pocket,cylinder(x,y,12,4,9));
 cut(part,pocket);
 // Closed capsule slot: 40 x 10, floor Z14.
 auto slot=fuse(box(80,43,14,30,10,7),cylinder(80,48,14,5,7));
 slot=fuse(slot,cylinder(110,48,14,5,7));cut(part,slot);
 for(const auto& p:{gp_Pnt(10,10,0),gp_Pnt(90,10,0),gp_Pnt(10,70,0),gp_Pnt(92,68,0)})cut(part,cylinder(p.X(),p.Y(),-1,3,22));
 cut(part,cylinder(73,20,10,4,11)); // diameter 8, depth 10
 cut(part,cylinder(73,67,8,3,13));  // diameter 6, depth 12
 cut(part,cylinder(105,27,-1,3,22)); // stepped hole: through diameter 6
 cut(part,cylinder(105,27,16,6,5)); // counterbore diameter 12, depth 4
 TopTools_IndexedMapOfShape solids,faces;
 TopExp::MapShapes(part,TopAbs_SOLID,solids);TopExp::MapShapes(part,TopAbs_FACE,faces);
 if(solids.Extent()!=1 || !BRepCheck_Analyzer(part).IsValid())throw std::runtime_error("expected one valid solid");
 const double pi=std::numbers::pi;
 const double expected=(120*80-20*10-15*15*.5)*20-(45*35-(4-pi)*16)*8-(30*10+25*pi)*6-5*pi*9*20-pi*16*10-pi*9*12-pi*(36-9)*4;
 if(std::abs(volume(part)-expected)>1e-5)throw std::runtime_error("analytic volume mismatch");
 Interface_Static::SetCVal("write.step.unit","MM");
 STEPControl_Writer writer;
 if(writer.Transfer(part,STEPControl_AsIs)!=IFSelect_RetDone || writer.Write(argv[1])!=IFSelect_RetDone)throw std::runtime_error("STEP export failed");
 STEPControl_Reader reader;
 if(reader.ReadFile(argv[1])!=IFSelect_RetDone || reader.TransferRoots()!=1)throw std::runtime_error("STEP reimport failed");
 auto readback=reader.OneShape();TopTools_IndexedMapOfShape readSolids;TopExp::MapShapes(readback,TopAbs_SOLID,readSolids);
 if(readSolids.Extent()!=1 || !BRepCheck_Analyzer(readback).IsValid() || std::abs(volume(readback)-expected)>1e-4)throw std::runtime_error("STEP roundtrip validation failed");
 std::cout<<std::setprecision(15)<<"VALIDATED solids="<<solids.Extent()<<" faces="<<faces.Extent()<<" volume_mm3="<<volume(readback)<<" expected_mm3="<<expected<<std::endl;
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
