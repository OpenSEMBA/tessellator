#include "StaircaseMesher.h"

#include <iostream>


#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Staircaser.h"
#include "core/Compressor.h"

#include "cgal/filler/Filler.h"
// #include "cgal/Manifolder.h"

#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;

StaircaseMesher::StaircaseMesher(const Mesh& inputMesh, int decimalPlacesInCollapser,  StaircaseMesherOptions opts) :
    MesherBase(inputMesh),
    decimalPlacesInCollapser_(decimalPlacesInCollapser),
    opts_(opts)
{
    log("Preparing surfaces.");
    surfaceMesh_ = MesherBase::buildSurfaceMesh(inputMesh, opts_.volumeGroups);
    log("Processing surface mesh.");
    process(surfaceMesh_);

    log("Preparing volumes");
    volumeMesh_ = MesherBase::buildVolumeMesh(inputMesh, opts_.volumeGroups);
    fillMesh(volumeMesh_, opts_.volumeGroups);
    log("Processing volume mesh.");
    process(volumeMesh_);

    mergeMesh(surfaceMesh_, volumeMesh_);
    
    log("Mesh built succesfully.", 1);
}

void StaircaseMesher::fillMesh(Mesh& m, const std::set<GroupId>& volumeGroups){
    if (m.countElems() == 0) return;
    // auto mani = meshlib::cgal::Manifolder(m);
    // for (const auto& gId : volumeGroups){
    //     m.groups[gId].elements = mani.getClosedSurfacesMesh().groups[gId].elements;
    // }
    m.coordinates = utils::GridTools{m.grid}.absoluteToRelative(m.coordinates);
    m = meshlib::cgal::filler::Filler(m).getMeshFilling();
    m.coordinates = utils::GridTools{m.grid}.relativeToAbsolute(m.coordinates);
    
    // m = f.getMeshFilling();

    // auto filling = f.getMeshFilling();
    // mergeMesh(m, filling);
    // // auto dimensions = getHighestDimensionByGroup(m);
    // // RedundancyCleaner::removeOverlappedElementsByDimension(m, dimensions);

}


Mesh StaircaseMesher::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = buildMeshFilteringElements(inputMesh, isNotTetrahedron);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

void StaircaseMesher::process(Mesh& mesh) const
{
    
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (mesh.countElems() == 0) {
        // mesh.grid = slicingGrid;
        return;
    }

    auto dimensions = getHighestDimensionByGroup(mesh);

    log("Slicing.", 1);
    mesh.grid = slicingGrid;
    mesh = Slicer{ mesh, dimensions }.getMesh();
    
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    // meshlib::cgal::filler::Filler f{mesh};
    // auto filling = f.getMeshFilling();
    // mergeMesh(mesh, filling);


    log("Collapsing.", 1);
    mesh = Collapser(mesh, decimalPlacesInCollapser_, dimensions).getMesh();

    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
    
    log("Staircasing.", 1);
    mesh = Staircaser(mesh).getMesh();

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    log("Removing repeated and overlapping elements.", 1);   
    RedundancyCleaner::removeOverlappedElementsByDimension(mesh, dimensions);

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    if (opts_.compress) {
        log("Compressing surfaces.", 1);
        std::size_t beforeQuads = countMeshElementsIf(mesh, isQuad);
        std::size_t merged = Compressor::compressSurfacesInMesh(mesh);
        std::size_t afterQuads = countMeshElementsIf(mesh, isQuad);
        log("Compressed " + std::to_string(beforeQuads) + 
            " -> " + std::to_string(afterQuads) + 
            " quads (merged " + std::to_string(merged) + " surfaces)", 1);
        
        log("Compressing lines.", 1);
        std::size_t beforeLines = countMeshElementsIf(mesh, isLine);
        merged = Compressor::compressLinesInMesh(mesh, dimensions);
        std::size_t afterLines = countMeshElementsIf(mesh, isLine);
        log("Compressed " + std::to_string(beforeLines) + 
            " -> " + std::to_string(afterLines) + 
            " lines (merged " + std::to_string(merged) + " segments)", 1);
    }
    
    log("Recovering original grid size.", 1);
    reduceGrid(mesh, originalGrid_);

    log("Converting relative to absolute coordinates.", 1);
    utils::meshTools::convertToAbsoluteCoordinates(mesh);
    
    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

}


Mesh StaircaseMesher::mesh() const
{
    return surfaceMesh_;
}

}