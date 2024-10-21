#include "Driver.h"

#include "DriverBase.cpp"
#include "Slicer.h"
#include "Collapser.h"
#include "Smoother.h"
#include "Snapper.h"

#include "filler/Filler.h"
#include "cgal/Repairer.h"

#include "utils/Cleaner.h"

namespace meshlib {
namespace tessellator {

using namespace utils;
using namespace filler;

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

Driver::Driver(const Mesh& in, const DriverOptions& opts) :
    DriverBase::DriverBase(in),
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

Mesh Driver::buildSurfaceMesh(const Mesh& inputMesh, const std::set<GroupId>& volumeGroups)
{
    auto resultMesh = DriverBase::buildSurfaceMesh(inputMesh);
    for (const auto& gId : volumeGroups) {
        resultMesh.groups[gId].elements.clear();
    }
    return resultMesh;
}

void Driver::process(Mesh& mesh) const
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

Mesh Driver::mesh() const 
{
    log("Building primal mesh.");
    Mesh res{ volumeMesh_ };
    mergeMesh(res, surfaceMesh_);
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));
    
    reduceGrid(res, originalGrid_);
    Cleaner::cleanCoords(res);

    log("Primal mesh built succesfully.", 1);
    return res;
}

Filler Driver::fill(const std::vector<Priority>& groupPriorities) const
{
    log("Building primal filler.", 1);
    
    return Filler{ 
        reduceGrid(volumeMesh_, originalGrid_), 
        reduceGrid(surfaceMesh_, originalGrid_),
        groupPriorities 
    };
}

Filler Driver::dualFill(const std::vector<Priority>& groupPriorities) const
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
}
