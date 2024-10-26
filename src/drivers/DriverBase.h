#pragma once

#include "types/Mesh.h"
#include "DriverInterface.h"

namespace meshlib {
namespace drivers {

class DriverBase : public DriverInterface {
public:
    DriverBase(const Mesh& in);
    virtual ~DriverBase() = default;
    virtual Mesh mesh() const abstract = 0;

protected:
    virtual Mesh buildSurfaceMesh(const Mesh& inputMesh);
    virtual void process(Mesh&) const abstract = 0;

    static void log(const std::string& msg, std::size_t level = 0);
    static void logNumberOfQuads(std::size_t nQuads);
    static void logNumberOfTriangles(std::size_t nTris);
    static void logNumberOfLines(std::size_t nLines);
    static void logNumberOfNodes(std::size_t nNodes);
    static void logGridSize(const Grid& g);

    static Grid buildNonSlicingGrid(const Grid& primal, const Grid& enlarged);
    static Grid buildSlicingGrid(const Grid& primal, const Grid& enlarged);

    Mesh extractSurfaceFromVolumeMeshes(const Mesh& inputMesh);

    Grid originalGrid_;
    Grid enlargedGrid_;
};

}
}