#include "StructuredDriver.h"

#include <iostream>


#include "Slicer.h"
#include "Collapser.h"
#include "Structurer.h"
#include "Driver.cpp"

#include "cgal/Manifolder.h"

#include "utils/Cleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {

using namespace utils;
using namespace meshTools;
/*
void log(const std::string& msg, std::size_t level = 0)
{
    std::cout << "[Tessellator] ";
    for (std::size_t i = 0; i < level; i++) {
        std::cout << "-- ";
    }

    std::cout << msg << std::endl;
}

void logNumberOfTriangles(std::size_t nTris)
{
    std::stringstream msg;
    msg << "Mesh contains " << nTris << " triangles.";
    log(msg.str(), 2);
}
*/

void logNumberOfLines(std::size_t nLines)
{
    std::stringstream msg;
    msg << "Mesh contains " << nLines << " lines.";
    log(msg.str(), 2);
}

void logNumberOfQuads(std::size_t nQuads)
{
    std::stringstream msg;
    msg << "Mesh contains " << nQuads << " quads.";
    log(msg.str(), 2);
}
/*
void logGridSize(const Grid& g)
{
    std::stringstream msg;
    msg << "Grid size is "
        << g[0].size() - 1 << "x" << g[1].size() - 1 << "x" << g[2].size() - 1;
    log(msg.str(), 2);
}
*/
Mesh extractSurfaceFromVolumeMeshes(const Mesh& inputMesh) {
    return cgal::Manifolder{ buildMeshFilteringElements(inputMesh, isTetrahedron) }.getClosedSurfacesMesh();
}

Mesh buildSurfaceMesh(const Mesh& in, const Mesh & volumeSurface)
{
    auto resultMesh{ buildMeshFilteringElements(in, isNotTetrahedron) };
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

StructuredDriver::StructuredDriver(const Mesh& inputMesh, int decimalPlacesInCollapser) :
    originalGrid_{ inputMesh.grid },
    decimalPlacesInCollapser_(decimalPlacesInCollapser)
{
    logGridSize(inputMesh.grid);
    logNumberOfTriangles(inputMesh.countTriangles());

    enlargedGrid_ = getEnlargedGridIncludingAllElements(inputMesh);

    log("Preparing volume surface.");
    auto volumeSurface = extractSurfaceFromVolumeMeshes(inputMesh);

    log("Preparing surfaces.");
    surfaceMesh_ = buildSurfaceMesh(inputMesh, volumeSurface);

    log("Processing surface mesh.");
    process(surfaceMesh_);

    log("Initial hull mesh built succesfully.");
}

void StructuredDriver::process(Mesh& mesh) const
{
    
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (mesh.countElems() == 0) {
        mesh.grid = slicingGrid;
        return;
    }

    log("Slicing.", 1);
    mesh.grid = slicingGrid;
    mesh = Slicer{ mesh }.getMesh();
    
    logNumberOfTriangles(mesh.countTriangles());

    log("Collapsing.", 1);
    mesh = Collapser(mesh, decimalPlacesInCollapser_).getMesh();

    logNumberOfTriangles(mesh.countTriangles());

    log("Structuring.", 1);
    mesh = Structurer(mesh).getMesh();

    logNumberOfQuads(mesh.countQuads());
    logNumberOfLines(mesh.countLines());


    Cleaner::removeRepeatedElements(mesh);

    logNumberOfQuads(mesh.countQuads());
    logNumberOfLines(mesh.countLines());
}


Mesh StructuredDriver::mesh() const
{
    log("Building primal mesh.");
    Mesh resultMesh{ surfaceMesh_ };

    logNumberOfQuads(resultMesh.countQuads());
    logNumberOfLines(resultMesh.countLines());

    reduceGrid(resultMesh, originalGrid_);
    Cleaner::cleanCoords(resultMesh);

    log("Primal mesh built succesfully.", 1);
    return resultMesh;
}

}
}