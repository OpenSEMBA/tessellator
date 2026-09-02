#include "ConformalMesher.h"

#include "MesherBase.h"
#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Compressor.h"
#include "core/Smoother.h" 
#include "core/Snapper.h"
#include "core/Staircaser.h"
#include "core/VolumeShellExtractor.h"

#include "utils/GridTools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "utils/CoordGraph.h"
#include "utils/RedundancyCleaner.h"

#include <cmath>
#include <numeric>
#include <sstream>

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;

namespace {

std::vector<ElementsView> buildConnectedComponents(const Elements& elements)
{
    if (elements.empty()) {
        return {};
    }

    std::vector<ElementId> parent(elements.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto findRoot = [&](ElementId elementId, const auto& find) -> ElementId {
        if (parent[elementId] != elementId) {
            parent[elementId] = find(parent[elementId], find);
        }
        return parent[elementId];
    };
    const auto unite = [&](ElementId first, ElementId second) {
        first = findRoot(first, findRoot);
        second = findRoot(second, findRoot);
        if (first != second) {
            parent[second] = first;
        }
    };

    std::map<CoordinateId, ElementId> firstElementByVertex;
    for (ElementId elementId = 0; elementId < elements.size(); ++elementId) {
        for (CoordinateId vertex : elements[elementId].vertices) {
            const auto [found, inserted] = firstElementByVertex.emplace(
                vertex, elementId);
            if (!inserted) {
                unite(elementId, found->second);
            }
        }
    }

    std::map<ElementId, ElementsView> components;
    for (ElementId elementId = 0; elementId < elements.size(); ++elementId) {
        components[findRoot(elementId, findRoot)].push_back(&elements[elementId]);
    }

    std::vector<ElementsView> result;
    result.reserve(components.size());
    for (auto& entry : components) {
        result.push_back(std::move(entry.second));
    }
    return result;
}

std::size_t affineDimension(
    const ElementsView& component,
    const Coordinates& coordinates)
{
    IdSet vertexIds;
    for (const Element* element : component) {
        vertexIds.insert(element->vertices.begin(), element->vertices.end());
    }
    if (vertexIds.size() <= 1) {
        return 0;
    }

    const Coordinate& origin = coordinates[*vertexIds.begin()];
    Coordinate firstDirection;
    bool hasFirstDirection = false;
    for (CoordinateId vertex : vertexIds) {
        const Coordinate direction = coordinates[vertex] - origin;
        if (direction.norm() > Geometry::NORM_TOLERANCE) {
            firstDirection = direction;
            hasFirstDirection = true;
            break;
        }
    }
    if (!hasFirstDirection) {
        return 0;
    }

    Coordinate planeNormal;
    bool hasPlane = false;
    for (CoordinateId vertex : vertexIds) {
        const Coordinate normal = firstDirection ^ (coordinates[vertex] - origin);
        if (normal.norm() > Geometry::NORM_TOLERANCE) {
            planeNormal = normal;
            hasPlane = true;
            break;
        }
    }
    if (!hasPlane) {
        return 1;
    }

    for (CoordinateId vertex : vertexIds) {
        if (std::abs(planeNormal * (coordinates[vertex] - origin))
            > Geometry::NORM_TOLERANCE) {
            return 3;
        }
    }
    return 2;
}

std::set<Cell> cellsOccupiedByDegenerateVolumeComponents(
    const Mesh& mesh,
    const std::set<GroupId>& volumeGroups)
{
    const GridTools tools(mesh.grid);
    std::set<Cell> result;
    for (GroupId groupId : volumeGroups) {
        if (groupId >= mesh.groups.size()) {
            throw std::runtime_error("Volume group index is outside the mesh.");
        }
        for (const ElementsView& component :
            buildConnectedComponents(mesh.groups[groupId].elements)) {
            if (affineDimension(component, mesh.coordinates) == 3) {
                continue;
            }
            for (const Element* element : component) {
                Coordinate centroid;
                for (CoordinateId vertex : element->vertices) {
                    centroid += mesh.coordinates[vertex]
                        / double(element->vertices.size());
                }
                const auto touchingCells = tools.getTouchingCells(centroid);
                result.insert(touchingCells.begin(), touchingCells.end());
            }
        }
    }
    return result;
}

Elements triangulateVolumeElement(const Element& element)
{
    if (!element.isQuad()) {
        return {element};
    }

    const auto& vertices = element.vertices;
    const auto firstDiagonal = std::minmax(vertices[0], vertices[2]);
    const auto secondDiagonal = std::minmax(vertices[1], vertices[3]);
    if (firstDiagonal < secondDiagonal) {
        return {
            Element({vertices[0], vertices[1], vertices[2]}, Element::Type::Surface),
            Element({vertices[0], vertices[2], vertices[3]}, Element::Type::Surface)};
    }
    return {
        Element({vertices[1], vertices[2], vertices[3]}, Element::Type::Surface),
        Element({vertices[1], vertices[3], vertices[0]}, Element::Type::Surface)};
}

void normalizeVolumeGroups(
    Mesh& mesh,
    const std::set<GroupId>& volumeGroups)
{
    for (GroupId groupId : volumeGroups) {
        if (groupId >= mesh.groups.size()) {
            throw std::runtime_error("Volume group index is outside the mesh.");
        }
        Elements normalized;
        for (const ElementsView& component :
            buildConnectedComponents(mesh.groups[groupId].elements)) {
            std::set<IdSet> vertexSets;
            const bool isThreeDimensional =
                affineDimension(component, mesh.coordinates) == 3;
            for (const Element* element : component) {
                const Elements parts = isThreeDimensional
                    ? triangulateVolumeElement(*element)
                    : Elements{*element};
                for (const Element& part : parts) {
                    const IdSet vertices(part.vertices.begin(), part.vertices.end());
                    if (vertexSets.insert(vertices).second) {
                        normalized.push_back(part);
                    }
                }
            }
        }
        mesh.groups[groupId].elements = std::move(normalized);
    }
}

void restoreThreeDimensionalVolumeComponents(
    Mesh& mesh,
    const Mesh& conformalMesh,
    const std::set<GroupId>& volumeGroups)
{
    bool needsConformalCoordinates = false;
    for (GroupId groupId : volumeGroups) {
        for (const ElementsView& component :
            buildConnectedComponents(conformalMesh.groups[groupId].elements)) {
            if (affineDimension(component, conformalMesh.coordinates) == 3) {
                needsConformalCoordinates = true;
                break;
            }
        }
    }
    if (!needsConformalCoordinates) {
        return;
    }

    const CoordinateId coordinateOffset = mesh.coordinates.size();
    mesh.coordinates.insert(
        mesh.coordinates.end(),
        conformalMesh.coordinates.begin(), conformalMesh.coordinates.end());
    for (GroupId groupId : volumeGroups) {
        Elements restored;
        for (const ElementsView& component :
            buildConnectedComponents(mesh.groups[groupId].elements)) {
            if (affineDimension(component, mesh.coordinates) != 3) {
                for (const Element* element : component) {
                    restored.push_back(*element);
                }
            }
        }
        for (const ElementsView& component :
            buildConnectedComponents(conformalMesh.groups[groupId].elements)) {
            if (affineDimension(component, conformalMesh.coordinates) != 3) {
                continue;
            }
            for (const Element* element : component) {
                restored.push_back(*element);
                for (CoordinateId& vertex : restored.back().vertices) {
                    vertex += coordinateOffset;
                }
            }
        }
        mesh.groups[groupId].elements = std::move(restored);
    }
    RedundancyCleaner::cleanCoords(mesh);
}

Elements closedSurfaceCore(const Elements& surfaceFaces)
{
    using Edge = std::pair<CoordinateId, CoordinateId>;
    std::vector<bool> retained(surfaceFaces.size(), true);
    bool changed = true;
    while (changed) {
        changed = false;
        std::map<Edge, std::size_t> edgeOccurrences;
        for (ElementId faceId = 0; faceId < surfaceFaces.size(); ++faceId) {
            if (!retained[faceId]) {
                continue;
            }
            const auto& vertices = surfaceFaces[faceId].vertices;
            for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
                ++edgeOccurrences[std::minmax(
                    vertices[vertex], vertices[(vertex + 1) % vertices.size()])];
            }
        }
        for (ElementId faceId = 0; faceId < surfaceFaces.size(); ++faceId) {
            if (!retained[faceId]) {
                continue;
            }
            const auto& vertices = surfaceFaces[faceId].vertices;
            for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
                if (edgeOccurrences[std::minmax(
                        vertices[vertex],
                        vertices[(vertex + 1) % vertices.size()])] == 1) {
                    retained[faceId] = false;
                    changed = true;
                    break;
                }
            }
        }
    }

    Elements result;
    for (ElementId faceId = 0; faceId < surfaceFaces.size(); ++faceId) {
        if (retained[faceId]) {
            result.push_back(surfaceFaces[faceId]);
        }
    }
    return result;
}

std::vector<Elements> splitAtNonManifoldEdges(const Elements& surfaceFaces)
{
    if (surfaceFaces.empty()) {
        return {};
    }

    using Edge = std::pair<CoordinateId, CoordinateId>;
    std::map<Edge, std::vector<ElementId>> edgeFaces;
    for (ElementId faceId = 0; faceId < surfaceFaces.size(); ++faceId) {
        const auto& vertices = surfaceFaces[faceId].vertices;
        for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
            edgeFaces[std::minmax(
                vertices[vertex], vertices[(vertex + 1) % vertices.size()])]
                .push_back(faceId);
        }
    }

    std::vector<ElementId> parent(surfaceFaces.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto findRoot = [&](ElementId faceId, const auto& find) -> ElementId {
        if (parent[faceId] != faceId) {
            parent[faceId] = find(parent[faceId], find);
        }
        return parent[faceId];
    };
    for (const auto& entry : edgeFaces) {
        const auto& faces = entry.second;
        if (faces.size() != 2) {
            continue;
        }
        const ElementId first = findRoot(faces[0], findRoot);
        const ElementId second = findRoot(faces[1], findRoot);
        if (first != second) {
            parent[second] = first;
        }
    }

    std::map<ElementId, Elements> regions;
    for (ElementId faceId = 0; faceId < surfaceFaces.size(); ++faceId) {
        regions[findRoot(faceId, findRoot)].push_back(surfaceFaces[faceId]);
    }
    std::vector<Elements> result;
    result.reserve(regions.size());
    for (auto& entry : regions) {
        result.push_back(std::move(entry.second));
    }
    return result;
}

void validateConformalVolumeComponents(
    const Mesh& mesh,
    const std::set<GroupId>& volumeGroups)
{
    for (GroupId groupId : volumeGroups) {
        if (groupId >= mesh.groups.size()) {
            throw std::runtime_error("Volume group index is outside the mesh.");
        }
        std::size_t componentId = 0;
        for (const ElementsView& component :
            buildConnectedComponents(mesh.groups[groupId].elements)) {
            if (affineDimension(component, mesh.coordinates) != 3) {
                ++componentId;
                continue;
            }

            Elements surfaceFaces;
            for (const Element* element : component) {
                if (element->isTriangle() || element->isQuad()) {
                    const Elements triangles = triangulateVolumeElement(*element);
                    surfaceFaces.insert(
                        surfaceFaces.end(), triangles.begin(), triangles.end());
                }
            }
            surfaceFaces = closedSurfaceCore(surfaceFaces);
            if (surfaceFaces.empty()) {
                std::stringstream message;
                message << "Invalid conformal volume group " << groupId
                    << " component " << componentId
                    << ": a three-dimensional component has no closed surface region.";
                throw std::runtime_error(message.str());
            }

            bool foundClosedRegion = false;
            for (const Elements& region : splitAtNonManifoldEdges(surfaceFaces)) {
                ElementsView regionView;
                for (const Element& face : region) {
                    regionView.push_back(&face);
                }
                if (affineDimension(regionView, mesh.coordinates) != 3) {
                    continue;
                }

                Mesh regionMesh{
                    mesh.grid,
                    mesh.coordinates,
                    Groups{Group{mesh.groups[groupId].name, region}}};
                try {
                    static_cast<void>(VolumeShellExtractor{regionMesh});
                    foundClosedRegion = true;
                } catch (const std::runtime_error&) {
                    // A region split off at a non-manifold junction can be a
                    // lower-dimensional remnant even when its vertices are not
                    // globally coplanar. It is valid as long as this component
                    // still contains at least one closed volume region.
                }
            }
            if (!foundClosedRegion) {
                std::stringstream message;
                message << "Invalid conformal volume group " << groupId
                    << " component " << componentId
                    << ": a three-dimensional component has no closed volume region.";
                throw std::runtime_error(message.str());
            }
            ++componentId;
        }
    }
}

using Edge = std::pair<CoordinateId, CoordinateId>;

Edge edgeKey(CoordinateId first, CoordinateId second)
{
    return std::minmax(first, second);
}

bool isAxisAlignedCellSizedQuad(
    const CoordinateIds& vertices,
    const Coordinates& coordinates,
    Axis normalAxis)
{
    constexpr double tolerance = 1e-9;
    const double planePosition = coordinates[vertices.front()][normalAxis];
    for (CoordinateId vertex : vertices) {
        if (std::abs(coordinates[vertex][normalAxis] - planePosition)
            > tolerance) {
            return false;
        }
    }

    for (Axis axis = X; axis <= Z; ++axis) {
        if (axis == normalAxis) {
            continue;
        }

        double lower = coordinates[vertices.front()][axis];
        double upper = lower;
        for (CoordinateId vertex : vertices) {
            lower = std::min(lower, coordinates[vertex][axis]);
            upper = std::max(upper, coordinates[vertex][axis]);
        }

        const double cellLower = std::round(lower);
        if (std::abs(lower - cellLower) > tolerance
            || std::abs(upper - (cellLower + 1.0)) > tolerance) {
            return false;
        }
        for (CoordinateId vertex : vertices) {
            const double position = coordinates[vertex][axis];
            if (std::abs(position - cellLower) > tolerance
                && std::abs(position - (cellLower + 1.0)) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool buildAxisAlignedCellSizedQuad(
    const Element& first,
    const Element& second,
    const Coordinates& coordinates,
    Element& quad)
{
    if (!first.isTriangle() || !second.isTriangle()) {
        return false;
    }

    const VecD firstNormal = Geometry::normal(
        Geometry::asTriV(first, coordinates));
    const VecD secondNormal = Geometry::normal(
        Geometry::asTriV(second, coordinates));
    if (firstNormal.norm() <= Geometry::NORM_TOLERANCE
        || secondNormal.norm() <= Geometry::NORM_TOLERANCE
        || firstNormal * secondNormal <= 0.0) {
        return false;
    }

    const VecD firstUnitNormal = firstNormal / firstNormal.norm();
    const VecD secondUnitNormal = secondNormal / secondNormal.norm();
    Axis normalAxis = X;
    for (Axis axis = Y; axis <= Z; ++axis) {
        if (std::abs(firstUnitNormal[axis])
            > std::abs(firstUnitNormal[normalAxis])) {
            normalAxis = axis;
        }
    }
    constexpr double tolerance = 1e-9;
    for (Axis axis = X; axis <= Z; ++axis) {
        if (axis != normalAxis
            && (std::abs(firstUnitNormal[axis]) > tolerance
                || std::abs(secondUnitNormal[axis]) > tolerance)) {
            return false;
        }
    }

    std::map<Edge, std::size_t> edgeOccurrences;
    std::vector<Edge> directedBoundary;
    for (const Element* triangle : {&first, &second}) {
        for (std::size_t vertex = 0; vertex < triangle->vertices.size(); ++vertex) {
            const Edge edge{
                triangle->vertices[vertex],
                triangle->vertices[(vertex + 1) % triangle->vertices.size()]};
            ++edgeOccurrences[edgeKey(edge.first, edge.second)];
            directedBoundary.push_back(edge);
        }
    }

    std::map<CoordinateId, CoordinateId> nextVertex;
    CoordinateIds vertices;
    for (const Edge& edge : directedBoundary) {
        if (edgeOccurrences[edgeKey(edge.first, edge.second)] == 1) {
            if (!nextVertex.emplace(edge.first, edge.second).second) {
                return false;
            }
            vertices.push_back(edge.first);
        }
    }
    if (vertices.size() != 4 || nextVertex.size() != 4) {
        return false;
    }

    CoordinateIds orderedVertices;
    orderedVertices.reserve(4);
    CoordinateId vertex = vertices.front();
    for (std::size_t i = 0; i < 4; ++i) {
        orderedVertices.push_back(vertex);
        const auto next = nextVertex.find(vertex);
        if (next == nextVertex.end()) {
            return false;
        }
        vertex = next->second;
    }
    if (vertex != orderedVertices.front()
        || !isAxisAlignedCellSizedQuad(
            orderedVertices, coordinates, normalAxis)) {
        return false;
    }

    quad = Element(orderedVertices, Element::Type::Surface);
    return true;
}

void mergeAxisAlignedCellSizedTrianglePairs(Mesh& mesh)
{
    for (Group& group : mesh.groups) {
        std::map<Edge, std::vector<ElementId>> trianglesByEdge;
        for (ElementId elementId = 0; elementId < group.elements.size(); ++elementId) {
            const Element& element = group.elements[elementId];
            if (!element.isTriangle()) {
                continue;
            }
            for (std::size_t vertex = 0; vertex < element.vertices.size(); ++vertex) {
                trianglesByEdge[edgeKey(
                    element.vertices[vertex],
                    element.vertices[(vertex + 1) % element.vertices.size()])]
                    .push_back(elementId);
            }
        }

        std::map<ElementId, Element> replacements;
        std::set<ElementId> removed;
        for (const auto& entry : trianglesByEdge) {
            const auto& triangles = entry.second;
            if (triangles.size() != 2
                || removed.count(triangles[0]) != 0
                || removed.count(triangles[1]) != 0) {
                continue;
            }

            Element quad;
            if (!buildAxisAlignedCellSizedQuad(
                    group.elements[triangles[0]], group.elements[triangles[1]],
                    mesh.coordinates, quad)) {
                continue;
            }
            const ElementId retained = std::min(triangles[0], triangles[1]);
            const ElementId discarded = std::max(triangles[0], triangles[1]);
            replacements.emplace(retained, std::move(quad));
            removed.insert(discarded);
        }

        if (replacements.empty()) {
            continue;
        }
        Elements normalized;
        normalized.reserve(group.elements.size() - removed.size());
        for (ElementId elementId = 0; elementId < group.elements.size(); ++elementId) {
            const auto replacement = replacements.find(elementId);
            if (replacement != replacements.end()) {
                normalized.push_back(replacement->second);
            } else if (removed.count(elementId) == 0) {
                normalized.push_back(group.elements[elementId]);
            }
        }
        group.elements = std::move(normalized);
    }
}

}

std::set<Cell> ConformalMesher::cellsWithMoreThanAVertexInsideEdge(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const GridTools gT(mesh.grid);
    std::set<std::pair<Cell, Axis>> ocuppiedEdges;
    for (const auto& v: mesh.coordinates) {
        if (gT.isRelativeInCellEdge(v)) {
            auto edge = std::make_pair(
                gT.toCell(v), 
                gT.getCellEdgeAxis(v).second);
            if (ocuppiedEdges.count(edge)) {
                auto touchingCells = gT.getTouchingCells(v);
                res.insert(touchingCells.begin(), touchingCells.end());
            } else {
                ocuppiedEdges.insert(edge);
            }
        }
    }

    return res;
}

std::map<Cell, std::vector<const Element*>> buildCellMapForAllElements(const Mesh& mesh)
{
    const auto gT = GridTools(mesh.grid);
    std::map<Cell, std::vector<const Element*>> res;
    for (auto const& g: mesh.groups) {
        auto groupElemMap = gT.buildCellElemMap(g.elements, mesh.coordinates);
        for (auto const& cellElemPair: groupElemMap) {
            res[cellElemPair.first].insert(
                res[cellElemPair.first].end(),
                cellElemPair.second.begin(),
                cellElemPair.second.end());
        }
    }
    return res;
}

std::size_t countPathsInCellBound(
    const GridTools& gT,
    const Mesh& mesh,
    const Cell& cell,
    const ElementsView& elementsInCell,
    const std::pair<Axis, Side>& bound)
{
    auto cG = CoordGraph(elementsInCell);
    auto vIds = cG.getVertices();
        
    std::size_t pathsInCellBound = 0;

    // Get vertices in the cell bound.
    IdSet vIdsInBound;
    for (auto const& vId: vIds) {
        if (gT.isRelativeAtCellBound(
            mesh.coordinates[vId], cell, bound)) {
            vIdsInBound.insert(vId);
        }
    }
    if (vIdsInBound.size() == 0) {
        return pathsInCellBound;
    }

    // Keep only edges if both vertices are in the cell bound.
    // Isolated vertices are not included.
    // Edges between two triangles are removed.
    CoordGraph cellBoundGraph;
    for (const auto& line: cG.getEdgesAsLines()) {
        const auto& v0 = line.vertices[0];
        const auto& v1 = line.vertices[1];
        if (!vIdsInBound.count(v0) || !vIdsInBound.count(v1)) {
            continue;
        }
        cellBoundGraph.addEdge(v0,v1);
    }
    Elements graphEdges = cellBoundGraph.getBoundaryGraph().getEdgesAsLines();
       
    // Count non-edge lines pointing outwards 
    // from each vertex in the edge.
    for (const auto& line: graphEdges) {
        const auto& c0 = mesh.coordinates[line.vertices[0]];
        const auto& c1 = mesh.coordinates[line.vertices[1]];
        if (!GridTools::areCoordOnSameEdge(c0, c1)) {
            pathsInCellBound++;
        }
    }
    return pathsInCellBound;
}

std::set<Cell> ConformalMesher::cellsWithMoreThanAPathPerFace(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const auto gT = GridTools(mesh.grid);
    
    for (auto const& c: buildCellMapForAllElements(mesh)) {
        for (Axis x: {X, Y, Z}) {
            for (Side s: {L, U}) {
                auto pathsInCellBound = countPathsInCellBound(gT, mesh, c.first, c.second, {x,s});
                if (pathsInCellBound > 1) {
                    res.insert(c.first);
                    Cell adjacentCell = c.first;
                    adjacentCell[x] = c.first[x] + (s == L ? -1 : 1);
                    res.insert(adjacentCell);
                }
            }
        }
    }
 
    return res;
}

std::set<Cell> ConformalMesher::cellsWithAVertexInAnEdgeForbiddenRegion(const Mesh& mesh)
{
    std::set<Cell> res;
    // TODO
    return res;
}

std::set<Cell> ConformalMesher::cellsSharedByGroups(
    const Mesh& mesh, const std::set<GroupId>& ignoredGroups)
{
    const GridTools gridTools(mesh.grid);
    std::map<Cell, std::set<GroupId>> groupsByCell;

    for (GroupId groupId = 0; groupId < mesh.groups.size(); ++groupId) {
        if (ignoredGroups.count(groupId) != 0) {
            continue;
        }
        const auto elementsByCell = gridTools.buildCellElemMap(
            mesh.groups[groupId].elements, mesh.coordinates);
        for (const auto& [cell, elements] : elementsByCell) {
            if (!elements.empty()) {
                groupsByCell[cell].insert(groupId);
            }
        }
    }

    std::set<Cell> sharedCells;
    for (const auto& [cell, groups] : groupsByCell) {
        if (groups.size() > 1) {
            sharedCells.insert(cell);
        }
    }
    return sharedCells;
}

std::set<Cell> mergeCellSets(const std::set<Cell>& a, const std::set<Cell>& b)
{
    std::set<Cell> res;
    res.insert(a.begin(), a.end());
    res.insert(b.begin(), b.end());
    return res;
}   

std::set<Cell> ConformalMesher::findNonConformalCells(const Mesh& mesh)
{
    // Find cells not respecting **The Three Rules**.
    std::set<Cell> res;
    
    // Rule #1: Cell edges must contain at most one vertex in each edge.
    res = mergeCellSets(res, cellsWithMoreThanAVertexInsideEdge(mesh));
    
    // Rule #2: Cell faces must always be crossed by a single path.
    res = mergeCellSets(res, cellsWithMoreThanAPathPerFace(mesh));

    // Rule #3: Conformal cells can't contain node or line elements.
    res = mergeCellSets(res, cellsContainingNodeOrLineElements(mesh));

    // Rule #4: Surface elements must preserve a consistent local orientation.
    res = mergeCellSets(res, cellsWithInvalidSurfaceOrientations(mesh));

    return res;
}

std::set<Cell> ConformalMesher::cellsWithInvalidSurfaceOrientations(
    const Mesh& mesh)
{
    GridTools gridTools(mesh.grid);
    std::set<Cell> result;
    for (const GroupElementId& location :
         utils::meshTools::getElementsWithInvalidSurfaceAdjacency(mesh)) {
        const Element& element = mesh.groups[location.first].elements[location.second];
        Coordinate centroid;
        for (CoordinateId vertex : element.vertices) {
            centroid += mesh.coordinates[vertex]
                / static_cast<double>(element.vertices.size());
        }
        const auto touchingCells = gridTools.getTouchingCells(centroid);
        result.insert(touchingCells.begin(), touchingCells.end());
    }
    return result;
}

std::set<Cell> ConformalMesher::cellsContainingNodeOrLineElements(const Mesh& mesh)
{
    GridTools gridTools(mesh.grid);
    std::set<Cell> result;
    for (const Group& group : mesh.groups) {
        const auto cellElements = gridTools.buildCellElemMap(
            group.elements, mesh.coordinates);
        for (const auto& entry : cellElements) {
            const bool containsLowerDimensionalElement = std::any_of(
                entry.second.begin(), entry.second.end(), [](const Element* element) {
                    return element->isNode() || element->isLine();
                });
            if (containsLowerDimensionalElement) {
                result.insert(entry.first);
            }
        }
    }
    return result;
}

Mesh ConformalMesher::mesh() const 
{
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    auto res = buildSurfaceMesh(inputMesh_, opts_.volumeGroups);
    auto volumes = buildVolumeMesh(inputMesh_, opts_.volumeGroups);
    if (!volumes.emptyOfElements()) {
        volumes = VolumeShellExtractor(volumes).getMesh();
    }
    mergeMesh(res, volumes);
    RedundancyCleaner::cleanCoords(res);

    if (res.countElems() == 0) {
        res.grid = slicingGrid;
        return res;
    }
    
    log("Slicing.", 1);
    res.grid = slicingGrid;
    res = Slicer{ res }.getMesh();
        
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    log("Smoothing.", 1);
    SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
    res = Smoother{res, smootherOpts}.getMesh();
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    const Mesh conformalMeshBeforeSnapping = res;

    log("Snapping.", 1);
    const Snapper snapper(res, opts_.snapperOptions);
    res = snapper.getMesh();

    Mesh conformalMeshBeforeStructuring = res;
    try {
        validateConformalVolumeComponents(
            conformalMeshBeforeStructuring, opts_.volumeGroups);
    } catch (const std::runtime_error&) {
        conformalMeshBeforeStructuring = conformalMeshBeforeSnapping;
    }

    log("Retriangulating planar patches.", 1);
    res = Smoother::retriangulatePlanarPatches(
        res, smootherOpts.featureDetectionAngle);
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    // Find cells which break conformal FDTD rules.
    auto nonConformalCells = findNonConformalCells(res);
    nonConformalCells = mergeCellSets(
        nonConformalCells, snapper.getCellsToStaircase());
    nonConformalCells = mergeCellSets(
        nonConformalCells,
        cellsOccupiedByDegenerateVolumeComponents(res, opts_.volumeGroups));
    log("Non-conformal cells found: " + std::to_string(nonConformalCells.size()), 1);

    // Calls structurer to mesh only those cells.
    log("Structuring non-conformal cells.", 1);
    res = Staircaser{res}.getSelectiveMesh(
        nonConformalCells, Staircaser::GapsFillingType::Insert);
    RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(res);
    RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(res);
    restoreThreeDimensionalVolumeComponents(
        res, conformalMeshBeforeStructuring, opts_.volumeGroups);
    normalizeVolumeGroups(res, opts_.volumeGroups);
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    // Find cells which break conformal FDTD rules after selective structuring.
    nonConformalCells = findNonConformalCells(res);
    log("Non-conformal cells found after Selective Structuring: " + std::to_string(nonConformalCells.size()), 1);

    const auto invalidOrientationCells =
        cellsWithInvalidSurfaceOrientations(res);
    if (!invalidOrientationCells.empty()) {
        log("Restaircasing cells with invalid surface orientation: "
            + std::to_string(invalidOrientationCells.size()), 1);
        res = Staircaser{res}.getSelectiveMesh(
            invalidOrientationCells, Staircaser::GapsFillingType::Insert);
        RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(
            res);
        RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(res);
        restoreThreeDimensionalVolumeComponents(
            res, conformalMeshBeforeStructuring, opts_.volumeGroups);
        normalizeVolumeGroups(res, opts_.volumeGroups);
    }


    if (opts_.mergeAxisAlignedTriangles) {
        log("Merging axis-aligned cell-sized triangle pairs.", 1);
        mergeAxisAlignedCellSizedTrianglePairs(res);
        RedundancyCleaner::fuseCoords(res);
        RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(res);
        RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(res);
        RedundancyCleaner::cleanCoords(res);
        logNumberOfQuads(countMeshElementsIf(res, isQuad));
    }

    if (opts_.compress && opts_.volumeGroups.empty()) {
        log("Compressing final mesh.", 1);
        Compressor::compressSurfacesInMesh(res);
        Compressor::compressLinesInMesh(res);
        RedundancyCleaner::fuseCoords(res);
        RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(res);
        RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(res);
        RedundancyCleaner::cleanCoords(res);
    }
    
    reduceGrid(res, originalGrid_);
    
    // Converts relatives to absolutes.
    utils::meshTools::convertToAbsoluteCoordinates(res);

    validateConformalVolumeComponents(res, opts_.volumeGroups);


    return res;
}

}
