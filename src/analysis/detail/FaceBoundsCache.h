#pragma once

#include "analysis/Model.h"
#include <TopTools_IndexedMapOfShape.hxx>

namespace oap::detail {
// Bind the stored coordinates to the geometry, placement and order of the imported faces.
std::string faceBoundsKey(const TopTools_IndexedMapOfShape& faces);
bool readFaceBounds(const std::string& path, const std::string& key, std::vector<Bbox>& bounds);
void writeFaceBounds(const std::string& path, const std::string& key, const std::vector<Bbox>& bounds);
}
