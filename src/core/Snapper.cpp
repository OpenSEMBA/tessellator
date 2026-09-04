#include "Snapper.h"

#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "Collapser.h"

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <optional>
#include <set>

#include <cmath>

namespace meshlib {
namespace core {

namespace {

using ElementLocation = std::pair<GroupId, ElementId>;

Coordinate surfaceNormal(const Element& element, const Coordinates& coordinates)
{
    return (coordinates[element.vertices[1]] - coordinates[element.vertices[0]])
        ^ (coordinates[element.vertices[2]] - coordinates[element.vertices[0]]);
}

std::set<ElementLocation> findElementsWithUnsafeNormals(
    const Mesh& mesh,
    const Coordinates& originalCoordinates)
{
    std::set<ElementLocation> unsafe;
    for (GroupId groupId = 0; groupId < mesh.groups.size(); ++groupId) {
        const Group& group = mesh.groups[groupId];
        for (ElementId elementId = 0;
             elementId < group.elements.size(); ++elementId) {
            const Element& element = group.elements[elementId];
            if (!element.isTriangle()) {
                continue;
            }
            const Coordinate originalNormal = surfaceNormal(
                element, originalCoordinates);
            const Coordinate snappedNormal = surfaceNormal(
                element, mesh.coordinates);
            if (originalNormal.norm() <= utils::Geometry::NORM_TOLERANCE) {
                continue;
            }
            if (snappedNormal.norm() > utils::Geometry::NORM_TOLERANCE
                && originalNormal * snappedNormal <= 0.0) {
                unsafe.insert({groupId, elementId});
            }
        }
    }
    return unsafe;
}

std::optional<CoordinateId> bestVertexToRestore(
    const Element& element,
    const Coordinates& snappedCoordinates,
    const Coordinates& originalCoordinates)
{
    const Coordinate originalNormal = surfaceNormal(
        element, originalCoordinates);
    std::optional<CoordinateId> result;
    double bestAlignment = -std::numeric_limits<double>::max();
    for (std::size_t vertexIndex = 0;
         vertexIndex < element.vertices.size(); ++vertexIndex) {
        const CoordinateId vertex = element.vertices[vertexIndex];
        if (snappedCoordinates[vertex] == originalCoordinates[vertex]) {
            continue;
        }
        TriV trial{
            snappedCoordinates[element.vertices[0]],
            snappedCoordinates[element.vertices[1]],
            snappedCoordinates[element.vertices[2]],
        };
        trial[vertexIndex] = originalCoordinates[vertex];
        const Coordinate trialNormal = utils::Geometry::normal(trial);
        const double alignment = trialNormal.norm()
            <= utils::Geometry::NORM_TOLERANCE
            ? -std::numeric_limits<double>::max()
            : originalNormal * trialNormal
                / (originalNormal.norm() * trialNormal.norm());
        if (!result.has_value() || alignment > bestAlignment) {
            result = vertex;
            bestAlignment = alignment;
        }
    }
    return result;
}

} // namespace


Snapper::Snapper(
    const Mesh& mesh,
    const SnapperOptions& opts,
    SurfaceInversionPolicy surfaceInversionPolicy) :
    mesh_{ mesh },
    opts_{ opts },
    surfaceInversionPolicy_{ surfaceInversionPolicy }
{
    if (!std::isfinite(opts.forbiddenLength) ||
        opts.forbiddenLength < 0.0 || opts.forbiddenLength > 0.5) {
        throw std::logic_error("Invalid forbidden length");
    }
    snap();
    
    mesh_ = Collapser{mesh_, 4}.getMesh();

    utils::meshTools::checkNoCellsAreCrossed(mesh_);
    utils::meshTools::checkNoNullAreasExist(mesh_);
}

std::pair<Coordinates, std::map<Coordinate, std::set<LinV>>> Snapper::buildListOfValidSolverPoints() const
{
    // Vertices
    Coordinates v(8);
    v[0] = VecD({ 0.0, 0.0, 0.0 });
    v[1] = VecD({ 1.0, 0.0, 0.0 });
    v[2] = VecD({ 1.0, 1.0, 0.0 });
    v[3] = VecD({ 0.0, 1.0, 0.0 });
    v[4] = VecD({ 0.0, 0.0, 1.0 });
    v[5] = VecD({ 1.0, 0.0, 1.0 });
    v[6] = VecD({ 1.0, 1.0, 1.0 });
    v[7] = VecD({ 0.0, 1.0, 1.0 });

    // Edges
    std::vector<LinV> edges(12);
    edges[0] = { v[0], v[1] };
    edges[1] = { v[1], v[2] };
    edges[2] = { v[2], v[3] };
    edges[3] = { v[3], v[0] };
    edges[4] = { v[0], v[4] };
    edges[5] = { v[1], v[5] };
    edges[6] = { v[2], v[6] };
    edges[7] = { v[3], v[7] };
    edges[8] = { v[4], v[5] };
    edges[9] = { v[5], v[6] };
    edges[10] = { v[6], v[7] };
    edges[11] = { v[7], v[4] };

    std::map<Coordinate, std::set<LinV>> coordsToEdge;

    std::set<Coordinate> aux;
    for (auto const& e : edges) {
        const Coordinate& vIni = e[0];
        aux.insert(vIni);
        coordsToEdge[vIni].insert(e);
        const Coordinate& vEnd = e[1];
        aux.insert(vEnd);
        coordsToEdge[vEnd].insert(e);

        const Coordinate vRIni = vIni + (vEnd - vIni) * opts_.forbiddenLength;
        aux.insert(vRIni);
        coordsToEdge[vRIni].insert(e);
        const Coordinate vREnd = vIni + (vEnd - vIni) * (1.0 - opts_.forbiddenLength);
        aux.insert(vREnd);
        coordsToEdge[vREnd].insert(e);

        for (std::size_t i = 0; i < opts_.edgePoints; i++) {
            const double t = double(i + 1) / double(opts_.edgePoints + 1);
            const Coordinate inner = vRIni + (vREnd - vRIni) * t;
            aux.insert(inner);
            coordsToEdge[inner].insert(e);

        }
    }

    return std::make_pair(Coordinates(aux.begin(), aux.end()), coordsToEdge);
}

bool edgeIsCandidate(
    const LinV& edge,
    const Relative& rel,
    const utils::GridTools& gT)
{
    if (gT.isRelativeInCellFace(rel)) {
        const Axis& axis = gT.getCellFaceAxis(rel).second;
        return !(edge[0][axis] == 1 || edge[1][axis] == 1);
    }
    return true;
}


void Snapper::snap()
{
    const Coordinates originalCoordinates = mesh_.coordinates;
    Coordinates solverPoints;
    std::map<Coordinate, std::set<LinV>> coordsToEdge;
    std::tie(solverPoints, coordsToEdge) = buildListOfValidSolverPoints();
    // Snaps using closest point
    utils::GridTools gT(mesh_.grid);
    std::vector<Relative> r = mesh_.coordinates;

    std::map<LinV, Coordinates> edgeToSnappedCoords;

    for (std::size_t i = 0; i < r.size(); i++) {
        Relative rel = r[i];
        if (gT.isRelativeInCellEdge(rel)) {
            Coordinate closest, solverPoint;
            std::tie(closest, solverPoint) = findClosestSolverPoint(rel, solverPoints, gT);

            CoordinateId id = i;
            r[id] = closest;

            VecD cell = gT.toCell(rel).as<double>();
            for (const auto& edge : coordsToEdge[solverPoint]) {
                LinV l = { edge[0] + cell, edge[1] + cell };
                edgeToSnappedCoords[l].push_back(closest);
            }
        }
    }

    for (std::size_t i = 0; i < r.size(); i++) {
        Relative rel = r[i];
        if (!gT.isRelativeInCellEdge(rel) && !gT.isRelativeInCellCorner(rel)) {
            Coordinate closest, solverPoint;
            std::tie(closest, solverPoint) = findClosestSolverPoint(rel, solverPoints, gT);

            VecD cell = gT.toCell(r[i]).as<double>();
            double minDist = std::numeric_limits<double>::max();
            for (const auto& edge : coordsToEdge[solverPoint]) {
                if (edgeIsCandidate(edge, rel, gT)) {
                    LinV l = { edge[0] + cell, edge[1] + cell };
                    for (const auto& c : edgeToSnappedCoords[l]) {
                        double dist = (gT.getPos(rel) - gT.getPos(c)).norm();
                        if (dist < minDist) {
                            minDist = dist;
                            closest = c;
                        }
                    }
                }
            }
            CoordinateId id = i;
            r[id] = closest;
        }
    }

    mesh_.coordinates = r;
    if (surfaceInversionPolicy_ == SurfaceInversionPolicy::Reject) {
        rollbackUnsafeSurfaceSnaps(originalCoordinates);
    }

 }

void Snapper::rollbackUnsafeSurfaceSnaps(
    const Coordinates& originalCoordinates)
{
    utils::GridTools gridTools(mesh_.grid);
    Mesh originalMesh = mesh_;
    originalMesh.coordinates = originalCoordinates;
    const auto originalUnsafeAdjacency =
        utils::meshTools::getElementsWithInvalidSurfaceAdjacency(
            originalMesh, true);
    const std::size_t maxIterations = mesh_.coordinates.size() + 1;
    for (std::size_t iteration = 0; iteration < maxIterations; ++iteration) {
        const std::set<ElementLocation> unsafeNormals =
            findElementsWithUnsafeNormals(
            mesh_, originalCoordinates);
        std::set<ElementLocation> unsafeAdjacency =
            utils::meshTools::getElementsWithInvalidSurfaceAdjacency(mesh_, true);
        for (const ElementLocation& originalUnsafe : originalUnsafeAdjacency) {
            unsafeAdjacency.erase(originalUnsafe);
        }
        std::set<ElementLocation> unsafe = unsafeNormals;
        unsafe.insert(unsafeAdjacency.begin(), unsafeAdjacency.end());
        if (unsafe.empty()) {
            return;
        }

        std::set<CoordinateId> coordinatesToRestore;
        for (const auto& [groupId, elementId] : unsafe) {
            const Element& element = mesh_.groups[groupId].elements[elementId];
            for (const Coordinates* coordinates :
                 std::array<const Coordinates*, 2>{
                     &originalCoordinates, &mesh_.coordinates}) {
                Coordinate centroid;
                for (CoordinateId vertex : element.vertices) {
                    centroid += (*coordinates)[vertex]
                        / static_cast<double>(element.vertices.size());
                }
                const auto touchingCells = gridTools.getTouchingCells(centroid);
                cellsToStaircase_.insert(
                    touchingCells.begin(), touchingCells.end());
            }

            if (unsafeNormals.count({groupId, elementId}) != 0) {
                const auto vertex = bestVertexToRestore(
                    element, mesh_.coordinates, originalCoordinates);
                if (vertex.has_value()) {
                    coordinatesToRestore.insert(*vertex);
                }
            }
            else {
                std::optional<CoordinateId> mostDisplacedVertex;
                double greatestDisplacement = 0.0;
                for (CoordinateId vertex : element.vertices) {
                    const double displacement =
                        (mesh_.coordinates[vertex]
                            - originalCoordinates[vertex]).norm();
                    if (displacement > greatestDisplacement) {
                        greatestDisplacement = displacement;
                        mostDisplacedVertex = vertex;
                    }
                }
                if (mostDisplacedVertex.has_value()) {
                    coordinatesToRestore.insert(*mostDisplacedVertex);
                }
            }
        }
        if (coordinatesToRestore.empty()) {
            return;
        }
        for (CoordinateId vertex : coordinatesToRestore) {
            mesh_.coordinates[vertex] = originalCoordinates[vertex];
        }
    }
}

std::pair<Coordinate, Coordinate> Snapper::findClosestSolverPoint(
    const Relative& rel,
    const Coordinates& solverPoints,
    const utils::GridTools& gT) const
{
    Coordinate closest;
    double minDist = std::numeric_limits<double>::max();
    std::size_t minEdge = 0;
    Coordinate pos = gT.getPos(rel);
    for (std::size_t j = 0; j < solverPoints.size(); j++) {
        Relative cellGridPoint = solverPoints[j] + gT.toCell(rel).as<double>();
        double dist = (pos - gT.getPos(cellGridPoint)).norm();
        if (dist < minDist) {
            minDist = dist;
            closest = cellGridPoint;
            minEdge = j;
        }
    }
    return std::make_pair(closest, solverPoints[minEdge]);
}



}
}
