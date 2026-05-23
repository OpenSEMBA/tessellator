#include "Smoother.h"

#include "utils/RedundancyCleaner.h"
#include "utils/Geometry.h"
#include "utils/Tools.h"
#include "utils/MeshTools.h"
#include "Collapser.h"

#include <assert.h>
#include <algorithm>

#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif

namespace meshlib {
namespace core {

using namespace utils;
using namespace meshTools;


Smoother::Smoother(const Mesh& mesh, const SmootherOptions& opts) :
    opts_(opts)
{
    meshTools::checkNoCellsAreCrossed(mesh);

    mesh_ = mesh;
    redundancyCleaner::fuseCoords(mesh_);

    mesh_ = meshTools::duplicateCoordinatesUsedByDifferentGroups(mesh_);
    mesh_ = meshTools::duplicateCoordinatesSharedBySingleTrianglesVertex(mesh_);

    Mesh res = mesh_;
    SmootherTools sT(res.grid, res.coordinates);
    for (auto& g : res.groups) {
        auto const singularIds =
            sT.buildSingularIds(g.elements, mesh_.coordinates, opts_.featureDetectionAngle);

        std::vector<ElementsView> patchs;
        for (auto const& cell : sT.buildCellElemMap(g.elements, mesh_.coordinates)) {
            for (auto const& p :
                Geometry::buildDisjointSmoothSets(cell.second, mesh_.coordinates, opts_.featureDetectionAngle)) {
                patchs.push_back(p);
            }
        }

        std::for_each(patchs.begin(), patchs.end(), [&](auto& p) {
            sT.remeshBoundary(g.elements, res.coordinates, mesh_.coordinates, p);
        });

        std::for_each(patchs.begin(), patchs.end(), [&](auto& p) {
            sT.collapsePointsOnCellEdges(res.coordinates, p, singularIds, opts_.contourAlignmentAngle);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif
            patchs.begin(), patchs.end(), [&](auto& p) {
            sT.collapsePointsOnCellFaces(res.coordinates, p, singularIds);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif
            patchs.begin(), patchs.end(), [&](auto& p) {
            sT.collapsePointsOnFeatureEdges(res.coordinates, p, singularIds);
        });

        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::par,
#endif
            patchs.begin(), patchs.end(), [&](auto& p) {
            sT.collapseInteriorPointsToBound(res.coordinates, p);
        });
    }

    redundancyCleaner::fuseCoords(res);
    redundancyCleaner::removeDegenerateElements(res);
    res = buildMeshFilteringElements(res, isTriangle);
    redundancyCleaner::cleanCoords(res);
    mesh_ = res;

    Coordinates& cs = mesh_.coordinates;
    for (auto const& g : mesh_.groups) {
        cs = sT.collapsePointsOnContour(g.elements, cs, opts_.contourAlignmentAngle);
    }
    redundancyCleaner::fuseCoords(mesh_);
    redundancyCleaner::removeDegenerateElements(mesh_);
    redundancyCleaner::cleanCoords(mesh_);

    meshTools::checkNoCellsAreCrossed(mesh_);
}

}
}
