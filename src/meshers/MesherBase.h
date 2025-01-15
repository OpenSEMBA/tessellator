#pragma once

#include "types/Mesh.h"

namespace meshlib {
namespace meshers {

class MesherBase {
public:
    MesherBase(const Mesh& in);
    virtual ~MesherBase() = default;
    virtual Mesh mesh() const = 0;

protected:
    virtual Mesh buildSurfaceMesh(const Mesh& inputMesh);
    virtual void process(Mesh&) const = 0;

    static void log(const std::string& msg, std::size_t level = 0);
    static void logNumberOfQuads(std::size_t nQuads);
    static void logNumberOfTriangles(std::size_t nTris);
    static void logNumberOfLines(std::size_t nLines);
    static void logNumberOfNodes(std::size_t nNodes);
    static void logGridSize(const Grid& g);

    static Grid buildNonSlicingGrid(const Grid& primal, const Grid& enlarged);
    static Grid buildSlicingGrid(const Grid& primal, const Grid& enlarged);

    Grid originalGrid_;
    Grid enlargedGrid_;
};

}
}