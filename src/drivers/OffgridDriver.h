#pragma once

#include "cgal/filler/Filler.h"
#include "types/Mesh.h"
#include "DriverBase.h"
#include "OffgridDriverOptions.h"

namespace meshlib {
namespace drivers {

using namespace cgal;

class OffgridDriver : public DriverBase {
public:
    OffgridDriver(const Mesh& in, const OffgridDriverOptions& opts = OffgridDriverOptions());
    virtual ~OffgridDriver() = default;
    Mesh mesh() const;

    filler::Filler fill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;
    filler::Filler dualFill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;

private:
    OffgridDriverOptions opts_;

    Mesh volumeMesh_;
    Mesh surfaceMesh_;

    Mesh buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);
    void process(Mesh&) const;

};

}
}
