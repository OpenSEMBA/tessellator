#pragma once

#include "filler/Filler.h"
#include "types/Mesh.h"
#include "DriverBase.h"
#include "DriverOptions.h"

namespace meshlib {
namespace drivers {

class OffgridDriver : public DriverBase {
public:
    OffgridDriver(const Mesh& in, const DriverOptions& opts = DriverOptions());
    virtual ~OffgridDriver() = default;
    Mesh mesh() const;

    filler::Filler fill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;
    filler::Filler dualFill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;

private:
    DriverOptions opts_;

    Mesh volumeMesh_;
    Mesh surfaceMesh_;

    Mesh buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);
    void process(Mesh&) const;

};

}
}
