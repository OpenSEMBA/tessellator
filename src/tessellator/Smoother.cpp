#include "Smoother.h"

#include "utils/Cleaner.h"
#include "utils/Geometry.h"
#include "utils/Tools.h"
#include "utils/MeshTools.h"
#include "Collapser.h"

#include "cgal/Manifolder.h"

#include <CGAL/Polygon_mesh_processing/manifoldness.h>

#include <assert.h>
#include <algorithm>
#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif

namespace meshlib {
namespace tessellator {

using namespace utils;
using namespace meshTools;


Smoother::Smoother(const Mesh& mesh, const SmootherOptions& opts) :
    sT_(SmootherTools(mesh.grid)),
    opts_(opts)
{
    meshTools::checkNoCellsAreCrossed(mesh);

    mesh_ = mesh;
    mesh_ = meshTools::duplicateCoordinatesUsedByDifferentGroups(mesh_);
    
    mesh_ = cgal::Manifolder(mesh_).getSurfacesMesh();

    Mesh res = mesh_;
    for (auto& g : res.groups) {
        auto const singularIds = 
            sT_.buildSingularIds(g.elements, mesh_.coordinates, opts_.featureDetectionAngle);

        std::vector<ElementsView> ps;
        for (auto const& cell : sT_.buildCellElemMap(g.elements, mesh_.coordinates)) {
            for (auto const& p :
                Geometry::buildDisjointSmoothSets(cell.second, mesh_.coordinates, opts_.featureDetectionAngle)) {
                ps.push_back(p);
            }
        }

        std::for_each(ps.begin(), ps.end(), [&](auto& p) {
            sT_.remeshBoundary(g.elements, res.coordinates, mesh_.coordinates, p);
        });
                
        std::for_each(ps.begin(), ps.end(), [&](auto& p) {
            sT_.collapsePointsOnCellEdges(res.coordinates, p, singularIds, opts_.contourAlignmentAngle);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif
            ps.begin(), ps.end(), [&](auto& p) {
            sT_.collapsePointsOnCellFaces(res.coordinates, p, singularIds);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif
            ps.begin(), ps.end(), [&](auto& p) {
            sT_.collapsePointsOnFeatureEdges(res.coordinates, p, singularIds);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif      
            ps.begin(), ps.end(), [&](auto& p) {
            sT_.collapseInteriorPointsToBound(res.coordinates, p);
        });

    }
    
    Cleaner::fuseCoords(res);
    res = buildMeshFilteringElements(res, isTriangle);
    for (auto& g : res.groups) {
        auto aux{ 
            cgal::polyhedronTools::buildPolyhedronFromElements(res.coordinates, g.elements) };
        cgal::PMP::duplicate_non_manifold_vertices(aux);
        g.elements.clear();
        g.elements = cgal::polyhedronTools::buildElementsFromPolyhedron(res.coordinates, aux);
    }
    Cleaner::cleanCoords(res);
    mesh_ = res;


    Coordinates& cs = mesh_.coordinates;
    for (auto const& g : mesh_.groups) {
        cs = sT_.collapsePointsOnContour(g.elements, cs, opts_.contourAlignmentAngle);
    }
    Cleaner::fuseCoords(mesh_);

    meshTools::checkNoCellsAreCrossed(mesh_);
}

}
}