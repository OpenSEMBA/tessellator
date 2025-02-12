#include "OffgridMesher.h"

#include "MesherBase.h"
#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Smoother.h"
#include "core/Snapper.h"

#include "cgal/filler/Filler.h"
#include "cgal/Repairer.h"
#include "cgal/Manifolder.h"

#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace meshTools;
using namespace core;
using namespace cgal::filler;

Mesh extractSurfaceFromVolumeMeshes(const Mesh& inputMesh)
{
    return cgal::Manifolder{ buildMeshFilteringElements(inputMesh, isTetrahedron) }.getClosedSurfacesMesh();
}

Mesh buildVolumeMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups)
{
    Mesh resultMesh{ inputMesh.grid, inputMesh.coordinates };
    resultMesh.groups.resize(inputMesh.groups.size());
    for (const auto& gId : volumeGroups) {
        mergeGroup(resultMesh.groups[gId], inputMesh.groups[gId]);
    }
    resultMesh = cgal::repair(resultMesh);
    mergeMesh(
        resultMesh,
        extractSurfaceFromVolumeMeshes(inputMesh)
    );
    return resultMesh;
}

OffgridMesher::OffgridMesher(const Mesh& in, const OffgridMesherOptions& opts) :
    MesherBase::MesherBase(in),
    opts_{ opts }
{        
    log("Preparing volumes.");
    volumeMesh_ = buildVolumeMesh(in, opts_.volumeGroups);
        
    log("Preparing surfaces.");
    surfaceMesh_ = buildSurfaceMesh(in, opts_.volumeGroups);

    log("Processing volume mesh.");
    process(volumeMesh_);

    log("Processing surface mesh.");
    process(surfaceMesh_);

    log("Initial hull mesh built succesfully.");
}

Mesh OffgridMesher::buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups)
{
    auto resultMesh = MesherBase::buildSurfaceMesh(inputMesh);
    for (const auto& gId : volumeGroups) {
        resultMesh.groups[gId].elements.clear();
    }
    return resultMesh;
}

void OffgridMesher::process(Mesh& mesh) const
{
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (mesh.countElems() == 0) {
        mesh.grid = slicingGrid;
        return;
    }
    
    log("Slicing.", 1);
    bool fullSlicing{ 
        opts_.forceSlicing 
        || opts_.collapseInternalPoints 
        || opts_.snap 
    };
    if (fullSlicing) {
        mesh.grid = slicingGrid;
    }
    else {
        mesh.grid = buildNonSlicingGrid(originalGrid_, enlargedGrid_);
    }
    mesh = Slicer{ mesh }.getMesh();
    if (!fullSlicing) {
        mesh = setGrid(mesh, slicingGrid);
    }
    
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    log("Collapsing.", 1);
    mesh = Collapser(mesh, opts_.decimalPlacesInCollapser).getMesh();
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
        
    if (opts_.collapseInternalPoints || opts_.snap) {
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

Filler OffgridMesher::fill(const std::vector<Priority>& groupPriorities) const
{
    log("Building primal filler.", 1);
    
    return Filler{ 
        reduceGrid(volumeMesh_, originalGrid_), 
        reduceGrid(surfaceMesh_, originalGrid_),
        groupPriorities 
    };
}

Filler OffgridMesher::dualFill(const std::vector<Priority>& groupPriorities) const
{
    log("Building dual filler.", 1);
    const auto dGrid{ GridTools{ originalGrid_ }.getExtendedDualGrid() };

    return Filler{ 
        setGrid(volumeMesh_, dGrid),
        setGrid(surfaceMesh_, dGrid),
        groupPriorities 
    };
}

}
