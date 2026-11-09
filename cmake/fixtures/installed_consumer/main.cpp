#include <analysis/Utf8Path.h>
#include <analysis/ContentHash.h>
#include <toolpath/pocket/geometry/PocketRegionBuilder.h>
#include <toolpath/finishing/SurfaceFinishing.h>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <QString>

int main()
{
    const auto path = oap::utf8Path("installed-consumer.step");
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape();
    const auto hash = oap::sha256Hex(reinterpret_cast<const unsigned char*>(""), 0);
    // Even a rejected input links the real finishing implementation and its
    // replaceable OpenCAMLib dependency from the installed package.
    const auto rejected = oap::toolpath::SurfaceFinishing{}.generate({}, gp_Ax3{}, {});
    const auto qtText = QString::fromUtf8("installed consumer");
    return path.empty() || box.IsNull()
        || rejected.ok || rejected.error.empty() || qtText.isEmpty()
        || hash != "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        ? 1 : 0;
}
