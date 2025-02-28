#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"
#include "OffgridMesherOptions.h"

namespace meshlib::meshers {


class OffgridMesher : public MesherBase {
public:
    OffgridMesher(const Mesh& in, const OffgridMesherOptions& opts = OffgridMesherOptions());
    virtual ~OffgridMesher() = default;
    Mesh mesh() const;

private:
    OffgridMesherOptions opts_;

    Mesh volumeMesh_;
    Mesh surfaceMesh_;

    void process(Mesh&) const;

};

}
