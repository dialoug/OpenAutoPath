#pragma once

#include <string>

namespace oap { struct ThreeAxisWireTraceUseCaseResult; }

// Complete canonical text, with unordered face sets and feature records sorted.
// Keep the text alongside any external digest so mismatches remain inspectable.
std::string completeThreeAxisRecognitionSignature(
    const oap::ThreeAxisWireTraceUseCaseResult& result);

// --cached-model <directory> --snapshot <file> [--skip-holes]
int runRecognitionSampleBenchmark(int argc, char** argv);
