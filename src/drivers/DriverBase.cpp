#include "DriverBase.h"

#include <iostream>

#include "cgal/Manifolder.h"

#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {


using namespace utils;
using namespace meshTools;


void DriverBase::log(const std::string& msg, std::size_t level)
{
    std::cout << "[Tessellator] ";
    for (std::size_t i = 0; i < level; i++) {
        std::cout << "-- ";
    }

    std::cout << msg << std::endl;
}

void DriverBase::logNumberOfQuads(std::size_t nQuads)
{
    std::stringstream msg;
    msg << "Mesh contains " << nQuads << " quads.";
    log(msg.str(), 2);
}

void DriverBase::logNumberOfTriangles(std::size_t nTris)
{
    std::stringstream msg;
    msg << "Mesh contains " << nTris << " triangles.";
    log(msg.str(), 2);
}

void DriverBase::logNumberOfLines(std::size_t nLines)
{
    std::stringstream msg;
    msg << "Mesh contains " << nLines << " lines.";
    log(msg.str(), 2);
}


void DriverBase::logNumberOfNodes(std::size_t nNodes)
{
    std::stringstream msg;
    msg << "Mesh contains " << nNodes << " nodes.";
    log(msg.str(), 2);
}

void DriverBase::logGridSize(const Grid& g)
{
    std::stringstream msg;
    msg << "Grid size is "
        << g[0].size() - 1 << "x" << g[1].size() - 1 << "x" << g[2].size() - 1;
    log(msg.str(), 2);
}

Mesh DriverBase::extractSurfaceFromVolumeMeshes(const Mesh& inputMesh) {
    return cgal::Manifolder{ buildMeshFilteringElements(inputMesh, isTetrahedron) }.getClosedSurfacesMesh();
}

DriverBase::DriverBase(const Mesh& inputMesh) : originalGrid_{inputMesh.grid}{

    logGridSize(inputMesh.grid);
    logNumberOfTriangles(countMeshElementsIf(inputMesh, isTriangle));

    enlargedGrid_ = getEnlargedGridIncludingAllElements(inputMesh);
}

Mesh DriverBase::buildSurfaceMesh(const Mesh& inputMesh) {
    return buildMeshFilteringElements(inputMesh, isNotTetrahedron);
}



Grid DriverBase::buildNonSlicingGrid(const Grid& primal, const Grid& enlarged)
{
    assert(primal.size() >= 2);
    assert(enlarged.size() >= 2);

    const auto dualGrid{ GridTools{primal}.getExtendedDualGrid() };
    Grid resultGrid;
    for (const auto& x : { X,Y,Z }) {
        std::set<CoordinateDir> gP;
        gP.insert(primal[x].front());
        gP.insert(primal[x].back());
        gP.insert(dualGrid[x].front());
        gP.insert(dualGrid[x].back());
        gP.insert(enlarged[x].front());
        gP.insert(enlarged[x].back());
        resultGrid[x].insert(resultGrid[x].end(), gP.begin(), gP.end());
    }
    return resultGrid;
}

Grid DriverBase::buildSlicingGrid(const Grid& primal, const Grid& enlarged)
{
    const auto nonSlicing{ buildNonSlicingGrid(primal, enlarged) };
    Grid r;
    for (const auto& x : { X,Y,Z }) {
        std::set<CoordinateDir> gP(nonSlicing[x].begin(), nonSlicing[x].end());
        gP.insert(primal[x].begin(), primal[x].end());
        r[x].insert(r[x].end(), gP.begin(), gP.end());
    }
    return r;
}

}
}