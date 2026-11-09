#include "analysis/ModelIndex.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

oap::Model modelWithIds(const std::string& id) {
    oap::Model model;
    model.faces.resize(1);
    model.faces.front().face_id = id;
    model.edges.resize(1);
    model.edges.front().edge_id = id;
    model.wires.resize(1);
    model.wires.front().wire_id = id;
    return model;
}

bool matches(const oap::ModelIndex& index, const oap::Model& model,
             const std::string& id) {
    return index.face(id) == &model.faces.front() &&
           index.edge(id) == &model.edges.front() &&
           index.wire(id) == &model.wires.front();
}

bool missing(const oap::ModelIndex& index, const std::string& id) {
    return !index.face(id) && !index.edge(id) && !index.wire(id);
}

} // namespace

int main() {
    const auto original = modelWithIds("original");
    oap::ModelIndex index;
    index.build(original);
    if (!matches(index, original, "original") || !missing(index, "missing"))
        return 1;

    const char* kinds[] = {"face", "edge", "wire"};
    for (int kind = 0; kind < 3; ++kind) {
        auto invalid = modelWithIds("duplicate");
        switch (kind) {
        case 0: invalid.faces.push_back(invalid.faces.front()); break;
        case 1: invalid.edges.push_back(invalid.edges.front()); break;
        case 2: invalid.wires.push_back(invalid.wires.front()); break;
        }
        bool rejected = false;
        try {
            index.build(invalid);
        } catch (const std::invalid_argument& error) {
            const auto expected = std::string("duplicate ") + kinds[kind] +
                                  " ID: duplicate";
            rejected = std::string(error.what()).find(expected) != std::string::npos;
        }
        if (!rejected || !matches(index, original, "original") ||
            !missing(index, "duplicate")) {
            std::cerr << "FAIL: duplicate " << kinds[kind]
                      << " must be diagnosed without replacing any old mapping\n";
            return 1;
        }
    }

    const auto replacement = modelWithIds("replacement");
    index.build(replacement);
    if (!matches(index, replacement, "replacement") || !missing(index, "original"))
        return 1;
    const oap::Model empty;
    index.build(empty);
    if (!missing(index, "replacement"))
        return 1;
    std::cout << "PASS: model index identity and failure checks\n";
    return 0;
}
