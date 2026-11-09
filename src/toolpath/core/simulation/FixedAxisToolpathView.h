// Non-owning, synchronous input to fixed-axis material removal. The originating
// feature retains its operation, tool and publication semantics; the material
// kernel consumes only the authoritative execution moves and cutting axis.
#pragma once

#include <span>
#include <string_view>

#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathMoves.h"

namespace oap::toolpath {

// The ID and move storage must outlive the synchronous apply call. This view
// must not be retained by an asynchronous job or material checkpoint.
struct FixedAxisToolpathView {
    std::string_view operationId;
    gp_Vec outwardToolAxis;
    std::span<const ToolpathMove> moves;
};

} // namespace oap::toolpath
