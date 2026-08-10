#include "Smoother.h"

#include "utils/RedundancyCleaner.h"
#include "utils/Geometry.h"
#include "utils/Tools.h"
#include "utils/MeshTools.h"
#include "Collapser.h"

#include <assert.h>
#include <algorithm>
#include <map>
#include <set>

#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif

namespace meshlib {
namespace core {

using namespace utils;
using namespace meshTools;

namespace {

using GridEntity = std::pair<Cell, Axis>;

struct OwnedCoordinates {
    std::map<GridEntity, IdSet> edges;
    std::map<GridEntity, IdSet> faces;
    std::map<Cell, IdSet> interiors;
};

Cell canonicalCell(Cell cell, const SmootherTools& tools)
{
    for (Axis axis = X; axis <= Z; ++axis) {
        cell[axis] = std::max<CellDir>(0, std::min(cell[axis], tools.numCellsDir(axis) - 1));
    }
    return cell;
}

OwnedCoordinates buildOwnedCoordinates(
    const Coordinates& coordinates,
    const SmootherTools& tools)
{
    OwnedCoordinates owned;
    for (CoordinateId id = 0; id < coordinates.size(); ++id) {
        const auto& coordinate = coordinates[id];
        if (tools.isRelativeInCellCorner(coordinate)) {
            continue; // Grid corners are fixed throughout smoothing.
        }

        Cell cell = canonicalCell(tools.toCell(coordinate), tools);
        const auto edge = tools.getCellEdgeAxis(coordinate);
        if (edge.first) {
            owned.edges[{cell, edge.second}].insert(id);
            continue;
        }
        const auto face = tools.getCellFaceAxis(coordinate);
        if (face.first) {
            owned.faces[{cell, face.second}].insert(id);
            continue;
        }
        owned.interiors[cell].insert(id);
    }
    return owned;
}

SmootherTools::IncidentElements buildIncidentElements(const Elements& elements)
{
    SmootherTools::IncidentElements incident;
    for (const auto& element : elements) {
        for (const auto id : element.vertices) {
            incident[id].push_back(&element);
        }
    }
    return incident;
}

}

Smoother::Smoother(const Mesh& mesh, const SmootherOptions& opts) :
    sT_(SmootherTools(mesh.grid)),
    opts_(opts)
{
    meshTools::checkNoCellsAreCrossed(mesh);

    mesh_ = mesh;
    mesh_ = meshTools::duplicateCoordinatesUsedByDifferentGroups(mesh_);
    mesh_ = meshTools::duplicateCoordinatesSharedBySingleTrianglesVertex(mesh_);
    
    Mesh res = mesh_;
    for (auto& g : res.groups) {
        auto const singularIds = 
            sT_.buildSingularIds(g.elements, mesh_.coordinates, opts_.featureDetectionAngle);

        // Boundary remeshing is preprocessing.  Keep its smooth sets separate
        // from the ownership phases below: an element can touch several grid
        // entities, while a coordinate has exactly one canonical owner.
        std::vector<ElementsView> patchs;
        for (auto const& cell : sT_.buildCellElemMap(g.elements, mesh_.coordinates)) {
            for (auto const& p :
                Geometry::buildDisjointSmoothSets(cell.second, mesh_.coordinates, opts_.featureDetectionAngle)) {
                patchs.push_back(p);
            }
        }

        std::for_each(patchs.begin(), patchs.end(), [&](auto& p) {
            sT_.remeshBoundary(g.elements, res.coordinates, mesh_.coordinates, p);
        });

        const auto incidentElements = buildIncidentElements(g.elements);
        const auto owned = buildOwnedCoordinates(res.coordinates, sT_);
        IdSet edgeIds, faceIds, interiorIds;
        for (const auto& entity : owned.edges) {
            edgeIds.insert(entity.second.begin(), entity.second.end());
        }
        for (const auto& entity : owned.faces) {
            faceIds.insert(entity.second.begin(), entity.second.end());
        }
        for (const auto& entity : owned.interiors) {
            interiorIds.insert(entity.second.begin(), entity.second.end());
        }

        // Later phases may read an earlier boundary but never move it.
        // This is intentionally sequential: entity ownership makes a future
        // conflict graph possible, but entities sharing an element still need
        // serialization until that scheduler is introduced.
        for (const auto& patch : patchs) {
            sT_.collapsePointsOnCellEdges(res.coordinates, patch, singularIds,
                opts_.contourAlignmentAngle, edgeIds);
            sT_.collapsePointsOnFeatureEdges(res.coordinates, patch, singularIds,
                incidentElements, edgeIds);
        }
        meshTools::checkNoCellsAreCrossed(res);

        for (const auto& patch : patchs) {
            sT_.collapsePointsOnCellFaces(res.coordinates, patch, singularIds, faceIds);
            sT_.collapsePointsOnFeatureEdges(res.coordinates, patch, singularIds,
                incidentElements, faceIds);
        }
        meshTools::checkNoCellsAreCrossed(res);

        for (const auto& patch : patchs) {
            sT_.collapsePointsOnFeatureEdges(res.coordinates, patch, singularIds,
                incidentElements, interiorIds);
            sT_.collapseInteriorPointsToBound(res.coordinates, patch, interiorIds);
        }
        meshTools::checkNoCellsAreCrossed(res);

    }
    
    RedundancyCleaner::fuseCoords(res);
    RedundancyCleaner::removeDegenerateElements(res);
    res = buildMeshFilteringElements(res, isTriangle);
    RedundancyCleaner::cleanCoords(res);
    mesh_ = res;


    Coordinates& cs = mesh_.coordinates;
    for (auto const& g : mesh_.groups) {
        cs = sT_.collapsePointsOnContour(g.elements, cs, opts_.contourAlignmentAngle);
    }
    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::removeDegenerateElements(mesh_);

    meshTools::checkNoCellsAreCrossed(mesh_);
}

}
}
