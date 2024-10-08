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


Mesh StructuredDriver::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = DriverBase::buildSurfaceMesh(inputMesh);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

StructuredDriver::StructuredDriver(const Mesh& inputMesh, int decimalPlacesInCollapser) :
    DriverBase(inputMesh),
    decimalPlacesInCollapser_(decimalPlacesInCollapser)
{
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
    logNumberOfLines(resultMesh.countNodes());

    reduceGrid(resultMesh, originalGrid_);
    Cleaner::cleanCoords(resultMesh);

    log("Primal mesh built succesfully.", 1);
    return resultMesh;
}

}
}