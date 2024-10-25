#pragma once

#include "types/Mesh.h"
#include "DriverInterface.h"

namespace meshlib {
namespace tessellator {

class DriverBase : public DriverInterface {
public:
    DriverBase(const Mesh& in);
    virtual ~DriverBase() = default;
    virtual Mesh mesh() const abstract = 0;

protected:
    virtual Mesh buildSurfaceMesh(const Mesh& inputMesh);
    virtual void process(Mesh&) const abstract = 0;


    Grid originalGrid_;
    Grid enlargedGrid_;
};

}
}