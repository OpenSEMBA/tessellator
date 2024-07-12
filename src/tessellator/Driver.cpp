#include "Driver.h"

#include <algorithm>
#include <stdexcept>
#include <assert.h>

#include "Slicer.h"
#include "Collapser.h"
#include "Smoother.h"
#include "Snapper.h"

#include "filler/Filler.h"
#include "cgal/Manifolder.h"
#include "cgal/Repairer.h"

#include "utils/Cleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {

using namespace utils;
using namespace meshTools;
using namespace filler;

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

void logGridSize(const Grid& g)
{
    std::stringstream msg;
    msg << "Grid size is "
        << g[0].size() - 1 << "x" << g[1].size() - 1 << "x" << g[2].size() - 1;
    log(msg.str(), 2);
}

Mesh buildVolumeMesh(const Mesh& in, const std::set<GroupId>& volumeGroups)
{
    Mesh r{ in.grid, in.coordinates };
    r.groups.resize(in.groups.size());
    for (const auto& gId : volumeGroups) {
        mergeGroup(r.groups[gId], in.groups[gId]);
    }
    r = cgal::repair(r);
    mergeMesh(
        r,
        Manifolder{ buildMeshFilteringElements(in, isTetrahedron) }.getClosedSurfacesMesh()
    );
    return r;
}

Mesh buildSurfaceMesh(const Mesh& in, const std::set<GroupId>& volumeGroups)
{
    auto r{ buildMeshFilteringElements(in, isNotTetrahedron) };
    for (const auto& gId : volumeGroups) {
        r.groups[gId].elements.clear();
    }
    return r;
}

Driver::Driver(const Mesh& in, const DriverOptions& opts) : 
    opts_{ opts },
    originalGrid_{in.grid}
{    
    logGridSize(in.grid);
    logNumberOfTriangles(in.countTriangles());
    
    enlargedGrid_ = getEnlargedGridIncludingAllElements(in) ;
    
    log("Preparing volumes.");
    vMesh_ = buildVolumeMesh(in, opts_.volumeGroups);
        
    log("Preparing surfaces.");
    sMesh_ = buildSurfaceMesh(in, opts_.volumeGroups);

    log("Processing volume mesh.");
    process(vMesh_);

    log("Processing surface mesh.");
    process(sMesh_);

    log("Initial hull mesh built succesfully.");
}

Grid buildNonSlicingGrid(const Grid& primal, const Grid& enlarged)
{
    assert(primal.size() >= 2);
    assert(enlarged.size() >= 2);
    
    const auto dual{ GridTools{primal}.getExtendedDualGrid() };
    Grid r;
    for (const auto& x : { X,Y,Z }) {
        std::set<CoordinateDir> gP;
        gP.insert(primal[x].front());
        gP.insert(primal[x].back());
        gP.insert(dual[x].front());
        gP.insert(dual[x].back());
        gP.insert(enlarged[x].front());
        gP.insert(enlarged[x].back());
        r[x].insert(r[x].end(), gP.begin(), gP.end());
    }
    return r;
}

Grid buildSlicingGrid(const Grid& primal, const Grid& enlarged)
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

void Driver::process(Mesh& m) const
{
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    if (m.countElems() == 0) {
        m.grid = slicingGrid;
        return;
    }
    
    log("Slicing.", 1);
    bool fullSlicing{ 
        opts_.forceSlicing 
        || opts_.collapseInternalPoints 
        || opts_.snap 
    };
    if (fullSlicing) {
        m.grid = slicingGrid;
    }
    else {
        m.grid = buildNonSlicingGrid(originalGrid_, enlargedGrid_);
    }
    m = Slicer{ m }.getMesh();
    if (!fullSlicing) {
        m = setGrid(m, slicingGrid);
    }
    
    logNumberOfTriangles(m.countTriangles());

    log("Collapsing.", 1);
    m = Collapser(m, opts_.decimalPlacesInCollapser).getMesh();
    logNumberOfTriangles(m.countTriangles());
        
    if (opts_.collapseInternalPoints || opts_.snap) {
        log("Smoothing.", 1);
        m = Smoother(m).getMesh();
        logNumberOfTriangles(m.countTriangles());
    }

    if (opts_.snap) {
        log("Snapping.", 1);
        m = Snapper(m, opts_.snapperOptions).getMesh();
        logNumberOfTriangles(m.countTriangles());
    }
}

Mesh Driver::mesh() const 
{
    log("Building primal mesh.");
    Mesh res{ vMesh_ };
    mergeMesh(res, sMesh_);
    logNumberOfTriangles(res.countTriangles());
    
    reduceGrid(res, originalGrid_);
    Cleaner::cleanCoords(res);

    log("Primal mesh built succesfully.", 1);
    return res;
}

Filler Driver::fill(const std::vector<Priority>& groupPriorities) const
{
    log("Building primal filler.", 1);
    
    return Filler{ 
        reduceGrid(vMesh_, originalGrid_), 
        reduceGrid(sMesh_, originalGrid_),
        groupPriorities 
    };
}

Filler Driver::dualFill(const std::vector<Priority>& groupPriorities) const
{
    log("Building dual filler.", 1);
    const auto dGrid{ GridTools{ originalGrid_ }.getExtendedDualGrid() };

    return Filler{ 
        setGrid(vMesh_, dGrid),
        setGrid(sMesh_, dGrid),
        groupPriorities 
    };
}

}
}
