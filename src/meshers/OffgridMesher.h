#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class OffgridMesherOptions {
public:
    bool forceSlicing = true;
    bool collapseInternalPoints = true;
    bool snap = true;
    core::SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 4;
    std::set<GroupId> volumeGroups{};

};

class OffgridMesher : public MesherBase {
public:
    OffgridMesher(const Mesh& in, const OffgridMesherOptions& opts = OffgridMesherOptions());
    virtual ~OffgridMesher() = default;
    Mesh mesh() const;

private:
    OffgridMesherOptions opts_;

    Mesh volumeMesh_;
    Mesh surfaceMesh_;

    Mesh buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);
    void process(Mesh&) const;

};

}
