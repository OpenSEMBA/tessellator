#include "OffgridMesher.h"

#include "MesherBase.h"
#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Smoother.h"
#include "core/Snapper.h"

#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace meshTools;
using namespace core;

OffgridMesher::OffgridMesher(const Mesh& in, const OffgridMesherOptions& opts) :
    MesherBase::MesherBase(in),
    opts_{ opts }
{        
    log("Retrieving groups to be meshed as volumes.");
    volumeMesh_ = buildVolumeMesh(in, opts_.volumeGroups);
    process(volumeMesh_);
        
    log("Retrieving groups to be meshed as surfaces.");
    surfaceMesh_ = buildSurfaceMesh(in, opts_.volumeGroups);  
    process(surfaceMesh_);

    log("Initial hull mesh built succesfully.");
}

void OffgridMesher::process(Mesh& mesh) const
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
    mesh = Collapser(mesh, opts_.decimalPlacesInCollapser).getMesh();
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
        
    if (opts_.smooth || opts_.snap) {
        log("Smoothing.", 1);
        mesh = Smoother(mesh).getMesh();
        logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
    }

    if (opts_.snap) {
        log("Snapping.", 1);
        mesh = Snapper(mesh, opts_.snapperOptions).getMesh();
        logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
    }
}

Mesh OffgridMesher::mesh() const 
{
    log("Building primal mesh.");
    Mesh res{ volumeMesh_ };
    mergeMesh(res, surfaceMesh_);
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));
    
    reduceGrid(res, originalGrid_);
    RedundancyCleaner::cleanCoords(res);

    log("Primal mesh built succesfully.", 1);
    return res;
}

}
