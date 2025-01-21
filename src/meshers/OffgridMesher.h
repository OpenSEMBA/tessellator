#pragma once

#include "cgal/filler/Filler.h"
#include "types/Mesh.h"
#include "MesherBase.h"
#include "OffgridMesherOptions.h"

namespace meshlib {
namespace meshers {

using namespace cgal;

class OffgridMesher : public MesherBase {
public:
    OffgridMesher(const Mesh& in, const OffgridMesherOptions& opts = OffgridMesherOptions());
    virtual ~OffgridMesher() = default;
    Mesh mesh() const;

    filler::Filler fill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;
    filler::Filler dualFill(
        const std::vector<Priority>& groupPriorities = std::vector<Priority>()) const;

private:
    OffgridMesherOptions opts_;

    Mesh volumeMesh_;
    Mesh surfaceMesh_;

    Mesh buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);
    void process(Mesh&) const;

};

}
}
