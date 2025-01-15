#include "StructuredMesher.h"

#include <iostream>


#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Structurer.h"

#include "utils/Cleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;


Mesh StructuredMesher::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = MesherBase::buildSurfaceMesh(inputMesh);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

StructuredMesher::StructuredMesher(const Mesh& inputMesh, int decimalPlacesInCollapser) :
    MesherBase(inputMesh),
    decimalPlacesInCollapser_(decimalPlacesInCollapser)
{
    log("Preparing surfaces.");
    surfaceMesh_ = buildMeshFilteringElements(inputMesh, isNotTetrahedron);

    log("Processing surface mesh.");
    process(surfaceMesh_);

    log("Initial hull mesh built succesfully.");
}

void StructuredMesher::process(Mesh& mesh) const
{
    
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (mesh.countElems() == 0) {
        mesh.grid = slicingGrid;
        return;
    }

    log("Slicing.", 1);
    mesh.grid = slicingGrid;
    mesh = Slicer{ mesh }.getMesh();
    
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    log("Collapsing.", 1);
    mesh = Collapser(mesh, decimalPlacesInCollapser_).getMesh();

    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    log("Structuring.", 1);
    mesh = Structurer(mesh).getMesh();

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));


    Cleaner::removeRepeatedElements(mesh);

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));
}


Mesh StructuredMesher::mesh() const
{
    log("Building primal mesh.");
    Mesh resultMesh{ surfaceMesh_ };

    logNumberOfQuads(countMeshElementsIf(resultMesh, isQuad));
    logNumberOfLines(countMeshElementsIf(resultMesh, isLine));
    logNumberOfNodes(countMeshElementsIf(resultMesh, isNode));

    reduceGrid(resultMesh, originalGrid_);
    Cleaner::cleanCoords(resultMesh);

    log("Primal mesh built succesfully.", 1);
    return resultMesh;
}

}