#pragma once

namespace oap::worker {
// Publish a phase transition on the job manager's stdout event stream.
void reportStage(const char* stage);
}
