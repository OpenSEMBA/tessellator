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




StructuredMesher::StructuredMesher(const Mesh& inputMesh, int decimalPlacesInCollapser) :
    MesherBase(inputMesh),
    decimalPlacesInCollapser_(decimalPlacesInCollapser)
{
    log("Preparing surfaces.");
    surfaceMesh_ = buildMeshFilteringElements(inputMesh, isNotTetrahedron);

    log("Processing surface mesh.");
    process(surfaceMesh_);
    
    log("Surface mesh built succesfully.", 1);
}

Mesh StructuredMesher::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = MesherBase::buildSurfaceMesh(inputMesh);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
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

    log("Removing repeated elements.", 1);   
    Cleaner::removeRepeatedElements(mesh);

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));
    
    log("Recovering original grid size.", 1);
    reduceGrid(mesh, originalGrid_);

    utils::GridTools gT{mesh.grid};
    mesh.coordinates = gT.relativeToAbsolute(mesh.coordinates);
    
    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

}


Mesh StructuredMesher::mesh() const
{
    return surfaceMesh_;
}

}