#include "RedundancyCleaner.h"

#include "Geometry.h"
#include "GridTools.h"

#include "MeshTools.h"

#include <map>
#include <set>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_set> 

namespace meshlib {
namespace utils {

void RedundancyCleaner::removeRepeatedElementsIgnoringOrientation(Mesh& m)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (const auto& g : m.groups) {
        auto gId{ &g - &m.groups.front() };
        std::map<IdSet, ElementId> vToE;
        for (const auto& e : g.elements) {
            auto eId{ &e - &g.elements.front() };
            IdSet vIds{ e.vertices.begin(), e.vertices.end() };
            if (vToE.count(vIds) == 0) {
                vToE.emplace(vIds, eId);
            }
            else {
                toRemove[gId].insert(eId);
            }
        }
    }

    removeElements(m, toRemove);
}

void RedundancyCleaner::removeRepeatedElements(Mesh& m)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (const auto& g : m.groups) {
        auto gId{&g - &m.groups.front()};
        std::map<CoordinateIds, ElementId> vToE;
        for (const auto& e : g.elements) {
            auto eId{ &e - &g.elements.front() };
            CoordinateIds vIds{ e.vertices };
            if (vIds.size() > 2) {
                std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
            }
            if (vToE.count(vIds) == 0) {
                vToE.emplace(vIds, eId);
            }
            else {
                toRemove[gId].insert(eId);
            }
        }
    }

    removeElements(m, toRemove);
}

void getOverlappedDimensionZeroElementsAndIdenticalLines(const Group& group, std::set<ElementId>& overlappedElements);
void getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(const Group& group, const std::vector<Coordinate>& meshCoordinates, std::set<ElementId>& overlappedElements);

namespace {

bool isPointOnSegment(
    const Coordinate& point,
    const Coordinate& first,
    const Coordinate& second)
{
    const Coordinate direction = second - first;
    const Coordinate pointDirection = point - first;
    const double directionSquared = direction * direction;
    if (directionSquared == 0.0) {
        return (point - first).norm() <= Geometry::NORM_TOLERANCE;
    }

    if ((direction ^ pointDirection).norm()
        > Geometry::NORM_TOLERANCE * std::sqrt(directionSquared)) {
        return false;
    }

    const double projection = direction * pointDirection;
    return projection >= -Geometry::NORM_TOLERANCE
        && projection <= directionSquared + Geometry::NORM_TOLERANCE;
}

bool arePointsCoincident(const Coordinate& first, const Coordinate& second)
{
    return (first - second).norm() <= Geometry::NORM_TOLERANCE;
}

bool isPointInTriangle(
    const Coordinate& point,
    const Coordinate& first,
    const Coordinate& second,
    const Coordinate& third)
{
    const Coordinate firstEdge = second - first;
    const Coordinate secondEdge = third - first;
    const Coordinate pointEdge = point - first;
    const Coordinate normal = firstEdge ^ secondEdge;
    const double normalSquared = normal * normal;
    if (normalSquared == 0.0
        || std::abs(normal * pointEdge)
            > Geometry::NORM_TOLERANCE * std::sqrt(normalSquared)) {
        return false;
    }

    const double dot00 = firstEdge * firstEdge;
    const double dot01 = firstEdge * secondEdge;
    const double dot02 = firstEdge * pointEdge;
    const double dot11 = secondEdge * secondEdge;
    const double dot12 = secondEdge * pointEdge;
    const double denominator = dot00 * dot11 - dot01 * dot01;
    if (std::abs(denominator) <= Geometry::NORM_TOLERANCE) {
        return false;
    }

    const double u = (dot11 * dot02 - dot01 * dot12) / denominator;
    const double v = (dot00 * dot12 - dot01 * dot02) / denominator;
    return u >= -Geometry::NORM_TOLERANCE
        && v >= -Geometry::NORM_TOLERANCE
        && u + v <= 1.0 + Geometry::NORM_TOLERANCE;
}

bool isPointInSurface(
    const Coordinate& point,
    const Element& surface,
    const Coordinates& coordinates)
{
    const auto& vertices = surface.vertices;
    if (surface.isTriangle()) {
        return isPointInTriangle(
            point,
            coordinates[vertices[0]],
            coordinates[vertices[1]],
            coordinates[vertices[2]]);
    }
    if (surface.isQuad()) {
        return isPointInTriangle(
                   point,
                   coordinates[vertices[0]],
                   coordinates[vertices[1]],
                   coordinates[vertices[2]])
            || isPointInTriangle(
                   point,
                   coordinates[vertices[0]],
                   coordinates[vertices[2]],
                   coordinates[vertices[3]]);
    }
    return false;
}

bool isLineInSurface(
    const Element& line,
    const Element& surface,
    const Coordinates& coordinates)
{
    return isPointInSurface(coordinates[line.vertices[0]], surface, coordinates)
        && isPointInSurface(coordinates[line.vertices[1]], surface, coordinates);
}

} // namespace

void RedundancyCleaner::removeOverlappedDimensionZeroElementsAndIdenticalLines(Mesh & mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];
        getOverlappedDimensionZeroElementsAndIdenticalLines(group, toRemove[g]);
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(
    Mesh& mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        const Group& group = mesh.groups[g];
        std::vector<const Element*> surfaces;
        std::vector<const Element*> lines;
        Coordinates nodeCoordinates;

        for (const Element& element : group.elements) {
            if (element.isTriangle() || element.isQuad()) {
                surfaces.push_back(&element);
            }
        }

        for (ElementId elementId = 0;
             elementId < group.elements.size(); ++elementId) {
            const Element& element = group.elements[elementId];
            if (element.isLine()) {
                const bool containedInSurface = std::any_of(
                    surfaces.begin(), surfaces.end(), [&](const Element* surface) {
                        return isLineInSurface(element, *surface, mesh.coordinates);
                    });
                if (containedInSurface) {
                    toRemove[g].insert(elementId);
                } else {
                    lines.push_back(&element);
                }
                continue;
            }
        }

        for (ElementId elementId = 0;
             elementId < group.elements.size(); ++elementId) {
            const Element& element = group.elements[elementId];
            if (!element.isNode()) {
                continue;
            }

            const Coordinate& coordinate = mesh.coordinates[element.vertices[0]];
            const bool containedInLine = std::any_of(
                lines.begin(), lines.end(), [&](const Element* line) {
                    return isPointOnSegment(
                        coordinate,
                        mesh.coordinates[line->vertices[0]],
                        mesh.coordinates[line->vertices[1]]);
                });
            const bool containedInSurface = std::any_of(
                surfaces.begin(), surfaces.end(), [&](const Element* surface) {
                    return isPointInSurface(coordinate, *surface, mesh.coordinates);
                });
            const bool coincidentWithNode = std::any_of(
                nodeCoordinates.begin(), nodeCoordinates.end(),
                [&](const Coordinate& nodeCoordinate) {
                    return arePointsCoincident(coordinate, nodeCoordinate);
                });
            if (containedInLine || containedInSurface || coincidentWithNode) {
                toRemove[g].insert(elementId);
            } else {
                nodeCoordinates.push_back(coordinate);
            }
        }
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(Mesh& mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto& group = mesh.groups[g];
        getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(group, mesh.coordinates, toRemove[g]);
    }

    removeElements(mesh, toRemove);
}

void getOverlappedDimensionZeroElementsAndIdenticalLines(
    const Group& group,
    std::set<ElementId>& overlappedElements)
{
    std::set<CoordinateId> usedCoordinates;
    std::vector<ElementId> nodesToCheck;

    for (std::size_t e = 0; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        if (element.isLine()) {
            usedCoordinates.insert(element.vertices[0]);
            usedCoordinates.insert(element.vertices[1]);
        }
        else if (element.isNode()) {
            nodesToCheck.push_back(e);
        }
    }

    for (auto e : nodesToCheck) {
        auto& node = group.elements[e];
        if (usedCoordinates.count(node.vertices[0]) == 0) {
            usedCoordinates.insert(node.vertices[0]);
        }
        else {
            overlappedElements.insert(e);
        }
    }
}

void getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(const Group& group, const std::vector<Coordinate> & meshCoordinates, std::set<ElementId>& overlappedElements)
{
    std::set<CoordinateIds> usedCoordinatesFromSurface;
    std::set<CoordinateIds> usedCoordinatePairsFromSurface;
    std::set<CoordinateId> usedCoordinates;
    std::vector<ElementId> linesToCheck;
    std::vector<ElementId> nodesToCheck;

    for (std::size_t e = 0; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        CoordinateIds vIds{ element.vertices };
        if (vIds.size() >= 2) {
            std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
            for (std::size_t v = 0; v < vIds.size(); ++v) {
                usedCoordinates.insert(vIds[v]);
            }
        }
        if (element.isQuad() || element.isTriangle()) {
            if (usedCoordinatesFromSurface.count(vIds) == 0) {
                usedCoordinatesFromSurface.insert(vIds);
                for (std::size_t v = 0; v < vIds.size(); ++v) {
                    auto firstCoordinateId = vIds[v];
                    auto secondCoordinateId = vIds[(v + 1) % vIds.size()];

                    if (secondCoordinateId < firstCoordinateId) {
                        std::swap(firstCoordinateId, secondCoordinateId);
                    }
                    usedCoordinatePairsFromSurface.insert({ firstCoordinateId, secondCoordinateId });
                }
            }
            else {
                overlappedElements.insert(e);
            }
        }
        else if (element.isLine()) {
            linesToCheck.push_back(e);
        }
        else if (element.isNode()) {
            nodesToCheck.push_back(e);
        }
    }

    std::map<CoordinateIds, ElementId> usedCoordinatePairsFromLine;

    for (auto e : linesToCheck) {
        auto& line = group.elements[e];
        CoordinateIds vIds{ line.vertices };
        std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());

        if (usedCoordinatePairsFromSurface.count(vIds)) {
            overlappedElements.insert(e);
        }
        else if (usedCoordinatePairsFromLine.count(vIds) == 0) {
            usedCoordinatePairsFromLine.emplace(vIds, e);
        }
        else {
            auto& originalLine = group.elements[usedCoordinatePairsFromLine[vIds]];
            RelativeDir direction = 0;
            RelativeDir originalDirection = 0;
            for (auto axis = X; axis <= Z; ++axis) {
                direction += meshCoordinates[line.vertices[1]][axis] - meshCoordinates[line.vertices[0]][axis];
                originalDirection += meshCoordinates[originalLine.vertices[1]][axis] - meshCoordinates[originalLine.vertices[0]][axis];
            }

            if (direction > originalDirection) {
                overlappedElements.insert(usedCoordinatePairsFromLine[vIds]);
                usedCoordinatePairsFromLine[vIds] = e;
            }
            else {
                overlappedElements.insert(e);
            }
        }
    }

    for (auto e : nodesToCheck) {
        auto& node = group.elements[e];
        if (usedCoordinates.count(node.vertices[0]) == 0) {
            usedCoordinates.insert(node.vertices[0]);
        }
        else {
            overlappedElements.insert(e);
        }
    }
}


void RedundancyCleaner::removeOverlappedElementsByDimension(Mesh& mesh, const std::vector<Element::Type>& highestDimensions)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];

        switch (highestDimensions[g]) {
            case Element::Type::Surface:
                getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(group, mesh.coordinates, toRemove[g]);
                break;
            case Element::Type::Line:
                getOverlappedDimensionZeroElementsAndIdenticalLines(group, toRemove[g]);
            default:
                break;
        }
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeElementsWithCondition(Mesh& m, std::function<bool(const Element&)> cnd)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (auto const& g : m.groups) {
        const GroupId gId = &g - &m.groups.front();
        for (auto const& e : g.elements) {
            const ElementId eId = &e - &g.elements.front();
            if (cnd(e)) {
                toRemove[gId].insert(eId);
            }
        }
    }
    removeElements(m, toRemove);
}

Elements RedundancyCleaner::findDegenerateElements_(
    const Group& g,
    const Coordinates& coords)
{
    Elements res;
    for (const auto e : g.elements) {
        if (!e.isTriangle()) {
            continue;
        }
        if (Geometry::isDegenerate(Geometry::asTriV(e, coords))) {
            res.push_back(e);
        }
    }
    return res;
}

void RedundancyCleaner::fuseCoords(Mesh& mesh) 
{
    std::map<Coordinate, IdSet> posIds;
    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            const Element& elem = mesh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId id = elem.vertices[i];
                Coordinate pos = mesh.coordinates[id];
                posIds[pos].insert(id);
            }
        }
    }

    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            Element& elem = mesh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId oldMeshedId = elem.vertices[i];
                CoordinateId newMeshedId = *posIds[mesh.coordinates[oldMeshedId]].begin();
                std::replace(elem.vertices.begin(), elem.vertices.end(), oldMeshedId, newMeshedId);
            }
        }
    }
}

void RedundancyCleaner::removeDegenerateElements(Mesh& mesh){
    removeElementsWithCondition(mesh, [&](const Element& e) {
        return IdSet(e.vertices.begin(), e.vertices.end()).size() != e.vertices.size();
    });
}

void RedundancyCleaner::cleanCoords(Mesh& output) 
{
    const std::size_t& numStrCoords = output.coordinates.size();

    IdSet coordsUsed;
    
    for (auto const& g: output.groups) {
        for (auto const& e: g.elements) {
                coordsUsed.insert(e.vertices.begin(), e.vertices.end());
        }
    }

    std::map<CoordinateId, CoordinateId> remap;
    std::vector<Coordinate> aux = output.coordinates;
    output.coordinates.clear();
    for (CoordinateId c = 0; c < aux.size(); c++) {
        if (coordsUsed.count(c) != 0) {
            remap[c] = output.coordinates.size();
            output.coordinates.push_back(aux[c]);
        }
        
    }
    for (GroupId g = 0; g < output.groups.size(); g++) {
        for (ElementId e = 0; e < output.groups[g].elements.size(); e++) {
            Element& elem = output.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                elem.vertices[i] = remap[elem.vertices[i]];
            }
            
        }
    }
    
}

void RedundancyCleaner::removeElements(Mesh& mesh, const std::vector<IdSet>& toRemove) 
{
    for (GroupId gId = 0; gId < mesh.groups.size(); gId++) {
        Elements& elems = mesh.groups[gId].elements;
        Elements newElems;
        newElems.reserve(elems.size() - toRemove[gId].size());
        auto it = toRemove[gId].begin();
        for (std::size_t i = 0; i < elems.size(); i++) {
            if (it == toRemove[gId].end() || i != *it) {
                newElems.push_back(elems[i]);
            }
            else {
                ++it;
            }
        }

        elems = newElems;       
    }
}

// The staircased surface can miss isolated unit faces on some grid rays,
// which leaves the surface open and makes VolumeFiller reject it because a
// ray has an odd number of crossings. This routine finds those rays (after
// discarding duplicated crossing pairs, which represent zero thickness
// pinches) and adds the missing unit face, inferring its grid plane from the
// crossings of the neighbouring rays.
void RedundancyCleaner::fillMissingUnitCellFaces(Mesh& mesh)
{
    using FaceKey = std::tuple<int, int, int, int>; // axis, plane, lower cell in the other two axes
    using EdgeKey = std::pair<Cell, Cell>;
    using RayKey = std::pair<int, int>;

    const auto cellOf = [&mesh](CoordinateId id, Cell& cell) {
        for (Axis axis : {X, Y, Z}) {
            const auto rounded = std::llround(mesh.coordinates[id][axis]);
            if (std::abs(mesh.coordinates[id][axis] - rounded) > 1e-9) {
                return false;
            }
            cell[axis] = static_cast<CellDir>(rounded);
        }
        return true;
    };
    const auto edgeKey = [](const Cell& first, const Cell& second) {
        return first < second ? std::make_pair(first, second)
                              : std::make_pair(second, first);
    };

    std::map<Cell, CoordinateId> coordinateIds;
    for (CoordinateId id = 0; id < mesh.coordinates.size(); ++id) {
        Cell cell;
        if (cellOf(id, cell)) {
            coordinateIds.emplace(cell, id);
        }
    }

    for (GroupId groupId = 0; groupId < mesh.groups.size(); ++groupId) {
        Group& group = mesh.groups[groupId];

        std::map<FaceKey, int> faceCount;
        std::map<EdgeKey, std::pair<Cell, Cell>> orientedEdges;

        for (const Element& element : group.elements) {
            if (!element.isQuad()) {
                continue;
            }

            std::array<Cell, 4> corners;
            bool allInteger = true;
            for (std::size_t i = 0; i < corners.size(); ++i) {
                if (!cellOf(element.vertices[i], corners[i])) {
                    allInteger = false;
                }
            }
            if (!allInteger) {
                continue;
            }

            std::vector<Axis> fixedAxes;
            for (Axis axis : {X, Y, Z}) {
                if (corners[0][axis] == corners[1][axis]
                    && corners[1][axis] == corners[2][axis]
                    && corners[2][axis] == corners[3][axis]) {
                    fixedAxes.push_back(axis);
                }
            }
            if (fixedAxes.size() != 1) {
                continue;
            }

            const Axis axis = fixedAxes.front();
            const Axis firstAxis = (axis + 1) % 3;
            const Axis secondAxis = (axis + 2) % 3;
            CellDir lower1 = corners[0][firstAxis];
            CellDir lower2 = corners[0][secondAxis];
            CellDir upper1 = lower1;
            CellDir upper2 = lower2;
            for (const Cell& corner : corners) {
                lower1 = std::min(lower1, corner[firstAxis]);
                upper1 = std::max(upper1, corner[firstAxis]);
                lower2 = std::min(lower2, corner[secondAxis]);
                upper2 = std::max(upper2, corner[secondAxis]);
            }
            if (upper1 - lower1 != 1 || upper2 - lower2 != 1) {
                continue;
            }

            ++faceCount[FaceKey{static_cast<int>(axis), corners[0][axis], lower1, lower2}];
            for (std::size_t i = 0; i < corners.size(); ++i) {
                const Cell& first = corners[i];
                const Cell& second = corners[(i + 1) % corners.size()];
                if (first == second) {
                    continue;
                }
                orientedEdges.emplace(edgeKey(first, second),
                    std::make_pair(first, second));
            }
        }

        std::array<std::map<RayKey, std::map<int, int>>, 3> rays;
        for (const auto& entry : faceCount) {
            const int axis = std::get<0>(entry.first);
            const int plane = std::get<1>(entry.first);
            const int lower1 = std::get<2>(entry.first);
            const int lower2 = std::get<3>(entry.first);
            rays[axis][{lower1, lower2}][plane] += entry.second;
        }

        std::vector<FaceKey> missingFaces;
        for (Axis axis : {X, Y, Z}) {
            const std::array<RayKey, 4> offsets{{{ -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 }}};
            for (const auto& ray : rays[axis]) {
                std::set<int> crossings;
                for (const auto& crossing : ray.second) {
                    if (crossing.second % 2 != 0) {
                        crossings.insert(crossing.first);
                    }
                }
                if (crossings.size() % 2 == 0) {
                    continue;
                }

                std::map<int, int> candidateScore;
                for (const RayKey& offset : offsets) {
                    const auto neighbor = rays[axis].find(
                        {ray.first.first + offset.first, ray.first.second + offset.second});
                    if (neighbor == rays[axis].end()) {
                        continue;
                    }
                    for (const auto& crossing : neighbor->second) {
                        if (crossing.second % 2 == 0) {
                            continue;
                        }
                        candidateScore[crossing.first - 1] += 1;
                        candidateScore[crossing.first] += 2;
                        candidateScore[crossing.first + 1] += 1;
                    }
                }

                bool found = false;
                int bestScore = -1;
                int inferredPlane = 0;
                for (const auto& candidate : candidateScore) {
                    bool nearExisting = false;
                    for (int crossing : crossings) {
                        if (std::abs(crossing - candidate.first) <= 1) {
                            nearExisting = true;
                            break;
                        }
                    }
                    if (nearExisting) {
                        continue;
                    }
                    if (candidate.second > bestScore
                        || (candidate.second == bestScore && candidate.first < inferredPlane)) {
                        bestScore = candidate.second;
                        inferredPlane = candidate.first;
                        found = true;
                    }
                }

                if (found) {
                    missingFaces.push_back(FaceKey{
                        static_cast<int>(axis), inferredPlane,
                        ray.first.first, ray.first.second});
                }
            }
        }

        for (const FaceKey& face : missingFaces) {
            const Axis axis = static_cast<Axis>(std::get<0>(face));
            const CellDir plane = std::get<1>(face);
            const CellDir lower1 = std::get<2>(face);
            const CellDir lower2 = std::get<3>(face);
            const Axis firstAxis = (axis + 1) % 3;
            const Axis secondAxis = (axis + 2) % 3;

            std::array<Cell, 4> corners;
            for (Cell& corner : corners) {
                corner[axis] = plane;
            }
            corners[0][firstAxis] = lower1;
            corners[0][secondAxis] = lower2;
            corners[1][firstAxis] = lower1 + 1;
            corners[1][secondAxis] = lower2;
            corners[2][firstAxis] = lower1 + 1;
            corners[2][secondAxis] = lower2 + 1;
            corners[3][firstAxis] = lower1;
            corners[3][secondAxis] = lower2 + 1;

            int bestMatches = std::numeric_limits<int>::min();
            std::array<Cell, 4> oriented = corners;
            for (int reverse = 0; reverse < 2; ++reverse) {
                for (int rotation = 0; rotation < 4; ++rotation) {
                    std::array<Cell, 4> candidate;
                    for (std::size_t i = 0; i < candidate.size(); ++i) {
                        const std::size_t index = (i + rotation) % candidate.size();
                        candidate[i] = reverse
                            ? corners[(corners.size() - index) % corners.size()]
                            : corners[index];
                    }
                    int matches = 0;
                    for (std::size_t i = 0; i < candidate.size(); ++i) {
                        const Cell& first = candidate[i];
                        const Cell& second = candidate[(i + 1) % candidate.size()];
                        const auto found = orientedEdges.find(edgeKey(first, second));
                        if (found == orientedEdges.end()) {
                            continue;
                        }
                        if (found->second.first == second && found->second.second == first) {
                            ++matches;
                        } else if (found->second.first == first && found->second.second == second) {
                            --matches;
                        }
                    }
                    if (matches > bestMatches) {
                        bestMatches = matches;
                        oriented = candidate;
                    }
                }
            }

            Element quad;
            quad.type = Element::Type::Surface;
            for (const Cell& corner : oriented) {
                const auto found = coordinateIds.find(corner);
                if (found != coordinateIds.end()) {
                    quad.vertices.push_back(found->second);
                    continue;
                }
                Coordinate coordinate;
                for (Axis a : {X, Y, Z}) {
                    coordinate[a] = corner[a];
                }
                const CoordinateId id = mesh.coordinates.size();
                mesh.coordinates.push_back(coordinate);
                coordinateIds.emplace(corner, id);
                quad.vertices.push_back(id);
            }
            group.elements.push_back(quad);
        }
    }
}

}
}
