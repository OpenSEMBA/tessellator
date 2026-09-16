#pragma once

#include "types/Mesh.h"
#include "MesherBaseOptions.h"

namespace meshlib {
namespace meshers {

class MesherBase {
public:
    MesherBase(const Mesh& in);
    virtual ~MesherBase() = default;
    virtual Mesh mesh() const = 0;
    const MesherBaseOptions & getOptions() const { return opts_; }

protected:
    virtual void process(Mesh&) const = 0;

    void log(const std::string& msg, std::size_t level = 0) const;
    void logNumberOfQuads(std::size_t nQuads) const;
    void logNumberOfTriangles(std::size_t nTris) const;
    void logNumberOfLines(std::size_t nLines) const;
    void logNumberOfNodes(std::size_t nNodes) const;
    void logNumberOfHexahedra(std::size_t nHexahedra) const;
    void logGridSize(const Grid& g) const;

    static Grid buildNonSlicingGrid(const Grid& primal, const Grid& enlarged);
    static Grid buildSlicingGrid(const Grid& primal, const Grid& enlarged);

    static Mesh buildVolumeMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);
    static Mesh buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups);

    Grid originalGrid_;
    Grid enlargedGrid_;
    MesherBaseOptions opts_;


};

}
}
