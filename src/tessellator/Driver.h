#pragma once

#include "filler/Filler.h"
#include "types/Mesh.h"
#include "DriverOptions.h"

namespace meshlib {
namespace tessellator {

class Driver {
public:
    Driver(const Mesh& in, const DriverOptions& opts = DriverOptions());
    virtual ~Driver() = default;
    Mesh mesh() const;

    filler::Filler fill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;
    filler::Filler dualFill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;

private:
    DriverOptions opts_;

    Mesh vMesh_;
    Mesh sMesh_;
    Grid originalGrid_;
    Grid enlargedGrid_;

    void process(Mesh&) const;

};

}
}
