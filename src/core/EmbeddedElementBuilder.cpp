#include "EmbeddedElementBuilder.h"

#include "utils/RedundancyCleaner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>

namespace meshlib {
namespace core {

namespace {

using EdgeKey = std::pair<CoordinateId, CoordinateId>;
using CellEdge = std::pair<Cell, Cell>;

constexpr std::size_t NO_ELEMENT = std::numeric_limits<std::size_t>::max();

EdgeKey edgeKey(CoordinateId first, CoordinateId second)
{
    return first < second
        ? EdgeKey{ first, second }
        : EdgeKey{ second, first };
}

CellEdge cellEdgeKey(const Cell& first, const Cell& second)
{
    return first < second
        ? CellEdge{ first, second }
        : CellEdge{ second, first };
}

std::string cellToString(const Cell& cell)
{
    std::ostringstream output;
    output << "(" << cell[0] << ", " << cell[1] << ", " << cell[2] << ")";
    return output.str();
}

std::string segmentToString(const EmbeddedElementBuilder::Segment& segment)
{
    return cellToString(segment[0]) + " -> " + cellToString(segment[1]);
}

Cell toCellOrThrow(const Coordinate& coordinate)
{
    Cell cell;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double rounded = std::round(coordinate[axis]);
        if (std::abs(coordinate[axis] - rounded) > 1e-5) {
            throw std::runtime_error(
                "Embedded geometry is not aligned with the relative grid: " +
                coordinate.str());
        }
        cell[axis] = static_cast<CellDir>(rounded);
    }
    return cell;
}

bool overlaps(const Cell& firstMin, const Cell& firstMax,
              const Cell& secondMin, const Cell& secondMax)
{
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (firstMax[axis] < secondMin[axis]
            || firstMin[axis] > secondMax[axis]) {
            return false;
        }
    }
    return true;
}

bool elementBounds(
    const Element& element,
    const Coordinates& coordinates,
    Cell& minimum,
    Cell& maximum)
{
    if (element.vertices.empty()) {
        return false;
    }

    for (std::size_t axis = 0; axis < 3; ++axis) {
        minimum[axis] = std::numeric_limits<CellDir>::max();
        maximum[axis] = std::numeric_limits<CellDir>::lowest();
    }

    for (const CoordinateId vertexId : element.vertices) {
        const Cell cell = toCellOrThrow(coordinates[vertexId]);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], cell[axis]);
            maximum[axis] = std::max(maximum[axis], cell[axis]);
        }
    }
    return true;
}

std::map<EdgeKey, std::size_t> buildEdgeFaceCounts(const Mesh& mesh)
{
    std::map<EdgeKey, std::set<std::vector<CoordinateId>>> facesByEdge;
    for (const Group& group : mesh.groups) {
        for (const Element& element : group.elements) {
            if (!element.isTriangle() && !element.isQuad()) {
                continue;
            }

            std::vector<CoordinateId> face = element.vertices;
            std::sort(face.begin(), face.end());

            for (std::size_t vertex = 0;
                 vertex < element.vertices.size();
                 ++vertex) {
                const CoordinateId first = element.vertices[vertex];
                const CoordinateId second = element.vertices[
                    (vertex + 1) % element.vertices.size()];
                facesByEdge[edgeKey(first, second)].insert(face);
            }
        }
    }

    std::map<EdgeKey, std::size_t> counts;
    for (const auto& entry : facesByEdge) {
        counts.emplace(entry.first, entry.second.size());
    }
    return counts;
}

std::map<Cell, CoordinateId> buildCellToId(const Mesh& mesh)
{
    std::map<Cell, CoordinateId> result;
    for (CoordinateId id = 0; id < mesh.coordinates.size(); ++id) {
        result.emplace(toCellOrThrow(mesh.coordinates[id]), id);
    }
    return result;
}

std::set<EdgeKey> quadEdgeSet(const Element& quad)
{
    std::set<EdgeKey> result;
    for (std::size_t vertex = 0; vertex < 4; ++vertex) {
        result.insert(edgeKey(
            quad.vertices[vertex],
            quad.vertices[(vertex + 1) % 4]));
    }
    return result;
}

bool shareEdge(const std::set<EdgeKey>& first, const std::set<EdgeKey>& second)
{
    for (const EdgeKey& edge : first) {
        if (second.count(edge) != 0) {
            return true;
        }
    }
    return false;
}

struct Transition {
    bool valid = false;
    int cost = 0;
    ElementId connector = NO_ELEMENT;
};

Transition findTransition(
    ElementId first,
    ElementId second,
    CoordinateId sharedVertex,
    const std::vector<Element>& quads,
    const std::vector<std::set<EdgeKey>>& quadEdges,
    const std::map<ElementId, std::set<ElementId>>& adjacency)
{
    if (first == second) {
        return { true, 0, NO_ELEMENT };
    }
    if (shareEdge(quadEdges[first], quadEdges[second])) {
        return { true, 0, NO_ELEMENT };
    }

    const auto firstIt = adjacency.find(first);
    const auto secondIt = adjacency.find(second);
    if (firstIt == adjacency.end() || secondIt == adjacency.end()) {
        return {};
    }

    // Both sides of an interior line can be valid. Ties are resolved by the
    // host element id, so the chosen side is deterministic for a given host
    // mesh but depends on the host element ordering.
    for (const ElementId candidate : firstIt->second) {
        if (candidate == first || candidate == second) {
            continue;
        }
        if (secondIt->second.count(candidate) == 0) {
            continue;
        }
        const auto& vertices = quads[candidate].vertices;
        if (std::find(vertices.begin(), vertices.end(), sharedVertex)
            == vertices.end()) {
            continue;
        }
        return { true, 1, candidate };
    }
    return {};
}

std::vector<ElementId> solveComponent(
    const std::vector<EmbeddedElementBuilder::Segment>& segments,
    const std::vector<std::vector<ElementId>>& candidates,
    const std::vector<Element>& quads,
    const std::vector<std::set<EdgeKey>>& quadEdges,
    const std::map<ElementId, std::set<ElementId>>& adjacency,
    const std::map<Cell, CoordinateId>& cellToId)
{
    const std::size_t segmentCount = segments.size();
    if (segmentCount == 0) {
        return {};
    }
    if (segmentCount == 1) {
        if (candidates[0].empty()) {
            throw std::runtime_error(
                "Embedded line segment is not embedded in a surface quad: " +
                segmentToString(segments[0]));
        }
        return { candidates[0].front() };
    }

    struct State {
        int cost = std::numeric_limits<int>::max();
        ElementId previous = NO_ELEMENT;
        ElementId connector = NO_ELEMENT;
    };

    const bool closed = segments.back()[1] == segments.front()[0];
    const std::size_t startCount = closed ? candidates.front().size() : 1;
    const auto startingCellIt = cellToId.find(segments.front()[0]);
    if (startingCellIt == cellToId.end()) {
        throw std::runtime_error(
            "Embedded line segment is not embedded in a surface: " +
            segmentToString(segments.front()));
    }
    const CoordinateId closingVertex = startingCellIt->second;

    std::vector<ElementId> bestSequence;
    int bestCost = std::numeric_limits<int>::max();

    for (std::size_t startIndex = 0; startIndex < startCount; ++startIndex) {
        std::vector<std::map<ElementId, State>> layers(segmentCount);

        if (closed) {
            const ElementId start = candidates.front().at(startIndex);
            layers[0].emplace(start, State{ 0, NO_ELEMENT, NO_ELEMENT });
        }
        else {
            for (const ElementId candidate : candidates.front()) {
                layers[0].emplace(candidate, State{ 0, NO_ELEMENT, NO_ELEMENT });
            }
        }

        bool possible = true;
        for (std::size_t i = 1; i < segmentCount; ++i) {
            const Cell& sharedCell = segments[i][0];
            const auto sharedIt = cellToId.find(sharedCell);
            if (sharedIt == cellToId.end()) {
                possible = false;
                break;
            }
            const CoordinateId sharedVertex = sharedIt->second;

            std::map<ElementId, State>& previousLayer = layers[i - 1];
            std::map<ElementId, State>& currentLayer = layers[i];
            for (const ElementId second : candidates[i]) {
                State best;
                for (const auto& entry : previousLayer) {
                    const Transition transition = findTransition(
                        entry.first, second, sharedVertex,
                        quads, quadEdges, adjacency);
                    if (!transition.valid) {
                        continue;
                    }
                    // Prefer keeping the same quad, then a direct edge
                    // neighbour, and only then inserting a connector quad.
                    // This avoids selecting redundant side quads at tight
                    // staircase corners, which would make the strip branch.
                    const int transitionCost = entry.first == second
                        ? 0
                        : (transition.connector == NO_ELEMENT ? 1 : 2);
                    const int cost = entry.second.cost + transitionCost;
                    if (cost < best.cost) {
                        best.cost = cost;
                        best.previous = entry.first;
                        best.connector = transition.connector;
                    }
                }
                if (best.cost != std::numeric_limits<int>::max()) {
                    currentLayer.emplace(second, best);
                }
            }
            if (currentLayer.empty()) {
                possible = false;
                break;
            }
        }
        if (!possible) {
            continue;
        }

        const std::map<ElementId, State>& finalLayer = layers.back();
        for (const auto& entry : finalLayer) {
            Transition closingTransition;
            int totalCost = entry.second.cost;
            if (closed) {
                closingTransition = findTransition(
                    entry.first,
                    candidates.front().at(startIndex),
                    closingVertex,
                    quads, quadEdges, adjacency);
                if (!closingTransition.valid) {
                    continue;
                }
                const ElementId closingQuad = candidates.front().at(startIndex);
                const int closingCost = entry.first == closingQuad
                    ? 0
                    : (closingTransition.connector == NO_ELEMENT ? 1 : 2);
                totalCost += closingCost;
            }
            if (totalCost >= bestCost) {
                continue;
            }

            std::vector<ElementId> choices(segmentCount, NO_ELEMENT);
            std::vector<ElementId> connectors(segmentCount, NO_ELEMENT);
            ElementId current = entry.first;
            choices.back() = current;
            for (std::size_t i = segmentCount - 1; i > 0; --i) {
                const State& state = layers[i].at(current);
                connectors[i] = state.connector;
                current = state.previous;
                choices[i - 1] = current;
            }

            std::vector<ElementId> sequence;
            for (std::size_t i = 0; i < segmentCount; ++i) {
                if (i > 0 && connectors[i] != NO_ELEMENT) {
                    sequence.push_back(connectors[i]);
                }
                sequence.push_back(choices[i]);
            }
            if (closed && closingTransition.connector != NO_ELEMENT) {
                sequence.push_back(closingTransition.connector);
            }

            bestCost = totalCost;
            bestSequence = std::move(sequence);
        }
    }

    if (bestSequence.empty() || bestCost == std::numeric_limits<int>::max()) {
        throw std::runtime_error(
            "Embedded line path cannot be represented as a continuous strip of "
            "surface quads without diagonal jumps.");
    }

    std::vector<ElementId> sequence;
    for (const ElementId elementId : bestSequence) {
        if (sequence.empty() || sequence.back() != elementId) {
            sequence.push_back(elementId);
        }
    }
    if (closed && sequence.size() > 1 && sequence.front() == sequence.back()) {
        sequence.pop_back();
    }
    return sequence;
}

std::vector<Cell> findHostPath(
    const Cell& start,
    const Cell& end,
    const std::vector<EmbeddedElementBuilder::Segment>& run,
    const std::map<Cell, std::set<Cell>>& adjacency)
{
    Cell minimum = run.front()[0];
    Cell maximum = run.front()[0];
    for (const auto& segment : run) {
        for (const Cell& vertex : segment) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                minimum[axis] = std::min(minimum[axis], vertex[axis]);
                maximum[axis] = std::max(maximum[axis], vertex[axis]);
            }
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        --minimum[axis];
        ++maximum[axis];
    }

    // Only allow short local detours. A staircase-phase mismatch normally
    // needs at most two extra unit edges to rejoin the host surface.
    const int maximumPathLength = static_cast<int>(run.size()) + 2;

    std::queue<Cell> pending;
    std::map<Cell, Cell> previous;
    std::map<Cell, int> distance;
    pending.push(start);
    distance.emplace(start, 0);

    bool found = false;
    while (!pending.empty()) {
        const Cell current = pending.front();
        pending.pop();
        if (current == end) {
            found = true;
            break;
        }
        if (distance[current] >= maximumPathLength) {
            continue;
        }

        const auto neighbours = adjacency.find(current);
        if (neighbours == adjacency.end()) {
            continue;
        }
        for (const Cell& next : neighbours->second) {
            bool insideBounds = true;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (next[axis] < minimum[axis] || next[axis] > maximum[axis]) {
                    insideBounds = false;
                    break;
                }
            }
            if (!insideBounds || distance.count(next) != 0) {
                continue;
            }
            distance.emplace(next, distance[current] + 1);
            previous.emplace(next, current);
            pending.push(next);
        }
    }

    if (!found) {
        return {};
    }

    std::vector<Cell> path;
    for (Cell current = end;; current = previous.at(current)) {
        path.push_back(current);
        if (current == start) {
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

EmbeddedElementBuilder::Components EmbeddedElementBuilder::buildOrderedUnitSegments(
    const Mesh& lineMesh)
{
    std::map<std::array<Cell, 2>, int> directedOccurrences;
    std::set<std::array<Cell, 2>> canonicalEdges;

    for (const Group& group : lineMesh.groups) {
        for (const Element& element : group.elements) {
            if (!element.isLine()) {
                continue;
            }

            const Cell first = toCellOrThrow(lineMesh.coordinates[element.vertices[0]]);
            const Cell second = toCellOrThrow(lineMesh.coordinates[element.vertices[1]]);

            std::vector<std::size_t> changedAxes;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (first[axis] != second[axis]) {
                    changedAxes.push_back(axis);
                }
            }
            if (changedAxes.empty()) {
                continue;
            }
            if (changedAxes.size() != 1) {
                throw std::runtime_error(
                    "Embedded line is not axis-aligned: " +
                    cellToString(first) + " -> " + cellToString(second));
            }

            const std::size_t axis = changedAxes.front();
            const int step = second[axis] > first[axis] ? 1 : -1;
            const int numberOfSteps = std::abs(second[axis] - first[axis]);
            Cell current = first;
            for (int i = 0; i < numberOfSteps; ++i) {
                Cell next = current;
                next[axis] += step;
                const Segment segment{ current, next };
                ++directedOccurrences[{ segment[0], segment[1] }];
                std::array<Cell, 2> key{ segment[0], segment[1] };
                if (key[1] < key[0]) {
                    std::swap(key[0], key[1]);
                }
                canonicalEdges.insert(key);
                current = next;
            }
        }
    }

    std::vector<Segment> segments;
    segments.reserve(canonicalEdges.size());
    for (const auto& key : canonicalEdges) {
        const int forward = directedOccurrences[{ key[0], key[1] }];
        const int backward = directedOccurrences[{ key[1], key[0] }];

        // Opposite traversals are staircase backtracks (for example a
        // one-cell spur at a corner). They carry no embedded area and would
        // otherwise make the line look branched. Same-direction repeats are
        // deduplicated; the majority direction keeps the surviving edge.
        if (forward > backward) {
            segments.push_back({ key[0], key[1] });
        }
        else if (backward > forward) {
            segments.push_back({ key[1], key[0] });
        }
    }

    std::map<Cell, std::vector<std::size_t>> incident;
    for (std::size_t segmentId = 0; segmentId < segments.size(); ++segmentId) {
        incident[segments[segmentId][0]].push_back(segmentId);
        incident[segments[segmentId][1]].push_back(segmentId);
    }

    std::vector<std::set<std::size_t>> neighbours(segments.size());
    for (const auto& entry : incident) {
        const std::vector<std::size_t>& ids = entry.second;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            for (std::size_t j = i + 1; j < ids.size(); ++j) {
                neighbours[ids[i]].insert(ids[j]);
                neighbours[ids[j]].insert(ids[i]);
            }
        }
    }

    std::set<std::size_t> visited;
    Components components;

    for (std::size_t startSegment = 0; startSegment < segments.size(); ++startSegment) {
        if (visited.count(startSegment) != 0) {
            continue;
        }

        std::set<std::size_t> componentIds;
        std::queue<std::size_t> pending;
        pending.push(startSegment);
        while (!pending.empty()) {
            const std::size_t current = pending.front();
            pending.pop();
            if (!componentIds.insert(current).second) {
                continue;
            }
            for (const std::size_t neighbour : neighbours[current]) {
                pending.push(neighbour);
            }
        }

        std::map<Cell, std::size_t> degree;
        for (const std::size_t segmentId : componentIds) {
            ++degree[segments[segmentId][0]];
            ++degree[segments[segmentId][1]];
        }
        for (const auto& entry : degree) {
            if (entry.second > 2) {
                throw std::runtime_error(
                    "Embedded line branches at grid vertex " +
                    cellToString(entry.first));
            }
        }

        std::set<std::size_t> remaining = componentIds;
        std::size_t currentSegment = *remaining.begin();
        Cell currentVertex = segments[currentSegment][0];

        for (const auto& entry : degree) {
            if (entry.second == 1) {
                currentVertex = entry.first;
                for (const std::size_t candidate : remaining) {
                    if (segments[candidate][0] == currentVertex
                        || segments[candidate][1] == currentVertex) {
                        currentSegment = candidate;
                        break;
                    }
                }
                break;
            }
        }

        std::vector<Segment> ordered;
        while (!remaining.empty()) {
            bool found = false;
            for (const std::size_t candidate : remaining) {
                if (segments[candidate][0] == currentVertex
                    || segments[candidate][1] == currentVertex) {
                    currentSegment = candidate;
                    found = true;
                    break;
                }
            }
            if (!found) {
                break;
            }

            Segment segment = segments[currentSegment];
            if (segment[0] != currentVertex) {
                std::swap(segment[0], segment[1]);
            }
            ordered.push_back(segment);
            remaining.erase(currentSegment);
            currentVertex = segment[1];

            std::vector<std::size_t> nextCandidates;
            for (const std::size_t candidate : remaining) {
                if (segments[candidate][0] == currentVertex
                    || segments[candidate][1] == currentVertex) {
                    nextCandidates.push_back(candidate);
                }
            }
            if (nextCandidates.size() > 1) {
                throw std::runtime_error(
                    "Embedded line branches at grid vertex " +
                    cellToString(currentVertex));
            }
            if (!nextCandidates.empty()) {
                currentSegment = nextCandidates.front();
            }
        }

        if (!remaining.empty()) {
            throw std::runtime_error(
                "Embedded line is not a single continuous polyline.");
        }

        visited.insert(componentIds.begin(), componentIds.end());
        components.push_back(std::move(ordered));
    }

    return components;
}

Mesh EmbeddedElementBuilder::buildCellAlignedSurface(
    const Mesh& hostSurface,
    const Components& components)
{
    if (components.empty()) {
        throw std::runtime_error("Embedded geometry has no line segments.");
    }

    Cell localMinimum(std::numeric_limits<CellDir>::max());
    Cell localMaximum(std::numeric_limits<CellDir>::lowest());
    for (const auto& component : components) {
        for (const Segment& segment : component) {
            for (const Cell& vertex : segment) {
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    localMinimum[axis] = std::min(localMinimum[axis], vertex[axis]);
                    localMaximum[axis] = std::max(localMaximum[axis], vertex[axis]);
                }
            }
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        localMinimum[axis] -= 2;
        localMaximum[axis] += 2;
    }

    Mesh result;
    result.grid = hostSurface.grid;
    result.groups.resize(1);
    result.groups[0].name = "surface";

    std::map<Cell, CoordinateId> cellToId;
    auto addVertex = [&result, &cellToId](const Cell& cell) {
        const auto existing = cellToId.find(cell);
        if (existing != cellToId.end()) {
            return existing->second;
        }
        Coordinate coordinate;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            coordinate[axis] = static_cast<CoordinateDir>(cell[axis]);
        }
        const CoordinateId id = result.coordinates.size();
        result.coordinates.push_back(coordinate);
        cellToId.emplace(cell, id);
        return id;
    };

    for (const Group& group : hostSurface.groups) {
        for (const Element& element : group.elements) {
            if (!element.isTriangle() && !element.isQuad()) {
                continue;
            }

            Cell elementMinimum;
            Cell elementMaximum;
            if (!elementBounds(
                    element, hostSurface.coordinates,
                    elementMinimum, elementMaximum)) {
                continue;
            }
            if (!overlaps(
                    elementMinimum, elementMaximum,
                    localMinimum, localMaximum)) {
                continue;
            }

            std::vector<Cell> cells;
            cells.reserve(element.vertices.size());
            for (const CoordinateId vertexId : element.vertices) {
                cells.push_back(toCellOrThrow(hostSurface.coordinates[vertexId]));
            }

            if (element.isTriangle()) {
                Element triangle;
                triangle.type = Element::Type::Surface;
                for (const Cell& cell : cells) {
                    triangle.vertices.push_back(addVertex(cell));
                }
                result.groups[0].elements.push_back(std::move(triangle));
                continue;
            }

            std::vector<std::size_t> constantAxes;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (elementMinimum[axis] == elementMaximum[axis]) {
                    constantAxes.push_back(axis);
                }
            }

            if (constantAxes.size() != 1) {
                Element quad;
                quad.type = Element::Type::Surface;
                for (const Cell& cell : cells) {
                    quad.vertices.push_back(addVertex(cell));
                }
                result.groups[0].elements.push_back(std::move(quad));
                continue;
            }

            const std::size_t planeAxis = constantAxes.front();
            const std::size_t firstAxis = (planeAxis + 1) % 3;
            const std::size_t secondAxis = (planeAxis + 2) % 3;
            const CellDir plane = cells.front()[planeAxis];

            const CellDir firstBegin = std::max(
                elementMinimum[firstAxis], localMinimum[firstAxis]);
            const CellDir firstEnd = std::min(
                elementMaximum[firstAxis], localMaximum[firstAxis]);
            const CellDir secondBegin = std::max(
                elementMinimum[secondAxis], localMinimum[secondAxis]);
            const CellDir secondEnd = std::min(
                elementMaximum[secondAxis], localMaximum[secondAxis]);

            long long signedArea = 0;
            for (std::size_t vertex = 0; vertex < 4; ++vertex) {
                const Cell& current = cells[vertex];
                const Cell& next = cells[(vertex + 1) % 4];
                signedArea += static_cast<long long>(current[firstAxis])
                        * next[secondAxis]
                    - static_cast<long long>(next[firstAxis])
                        * current[secondAxis];
            }
            if (signedArea == 0) {
                continue;
            }

            for (CellDir first = firstBegin; first < firstEnd; ++first) {
                for (CellDir second = secondBegin; second < secondEnd; ++second) {
                    std::array<std::pair<CellDir, CellDir>, 4> corners{ {
                        { first, second },
                        { first + 1, second },
                        { first + 1, second + 1 },
                        { first, second + 1 }
                    } };
                    if (signedArea < 0) {
                        std::swap(corners[1], corners[3]);
                    }

                    Element quad;
                    quad.type = Element::Type::Surface;
                    for (const auto& corner : corners) {
                        Cell cell;
                        cell[planeAxis] = plane;
                        cell[firstAxis] = corner.first;
                        cell[secondAxis] = corner.second;
                        quad.vertices.push_back(addVertex(cell));
                    }
                    result.groups[0].elements.push_back(std::move(quad));
                }
            }
        }
    }

    if (result.countElems() == 0) {
        throw std::runtime_error(
            "Embedded line is not embedded in any staircase surface.");
    }
    return result;
}

EmbeddedElementBuilder::Components
EmbeddedElementBuilder::snapComponentsToSurface(
    const Mesh& cellAlignedSurface,
    const Components& components)
{
    std::set<CellEdge> hostEdges;
    std::map<Cell, std::set<Cell>> adjacency;

    for (const Group& group : cellAlignedSurface.groups) {
        for (const Element& element : group.elements) {
            if (!element.isQuad()) {
                continue;
            }

            std::vector<Cell> cells;
            cells.reserve(element.vertices.size());
            for (const CoordinateId vertexId : element.vertices) {
                cells.push_back(
                    toCellOrThrow(cellAlignedSurface.coordinates[vertexId]));
            }

            for (std::size_t vertex = 0; vertex < cells.size(); ++vertex) {
                const Cell& first = cells[vertex];
                const Cell& second = cells[(vertex + 1) % cells.size()];
                if (first == second) {
                    continue;
                }
                hostEdges.insert(cellEdgeKey(first, second));
                adjacency[first].insert(second);
                adjacency[second].insert(first);
            }
        }
    }

    Components result;
    result.reserve(components.size());

    for (const auto& component : components) {
        if (component.empty()) {
            result.push_back(component);
            continue;
        }

        const bool closed = component.size() > 1
            && component.back()[1] == component.front()[0];

        std::vector<Segment> ordered = component;
        if (closed) {
            // Start at an embedded segment so that any missing run is linear
            // and has host vertices on both sides.
            const auto firstEmbedded = std::find_if(
                ordered.begin(), ordered.end(),
                [&hostEdges](const Segment& segment) {
                    return hostEdges.count(
                        cellEdgeKey(segment[0], segment[1])) != 0;
                });
            if (firstEmbedded != ordered.end()
                && firstEmbedded != ordered.begin()) {
                std::rotate(ordered.begin(), firstEmbedded, ordered.end());
            }
        }

        std::vector<Segment> repaired;
        repaired.reserve(ordered.size() + 4);

        std::size_t index = 0;
        while (index < ordered.size()) {
            const Segment& segment = ordered[index];
            if (hostEdges.count(cellEdgeKey(segment[0], segment[1])) != 0) {
                repaired.push_back(segment);
                ++index;
                continue;
            }

            const std::size_t runBegin = index;
            while (index < ordered.size()
                && hostEdges.count(cellEdgeKey(
                    ordered[index][0], ordered[index][1])) == 0) {
                ++index;
            }
            const std::size_t runEnd = index - 1;

            const Cell start = ordered[runBegin][0];
            const Cell end = ordered[runEnd][1];
            if (start != end
                && adjacency.count(start) != 0
                && adjacency.count(end) != 0) {
                const std::vector<Segment> run(
                    ordered.begin() + runBegin,
                    ordered.begin() + runEnd + 1);
                const std::vector<Cell> path =
                    findHostPath(start, end, run, adjacency);
                if (!path.empty()) {
                    for (std::size_t i = 1; i < path.size(); ++i) {
                        repaired.push_back({ path[i - 1], path[i] });
                    }
                    continue;
                }
            }

            // No local host path was found. Keep the original run so that
            // validateEmbedding reports the same error as before.
            for (std::size_t i = runBegin; i <= runEnd; ++i) {
                repaired.push_back(ordered[i]);
            }
        }

        result.push_back(std::move(repaired));
    }

    return result;
}

void EmbeddedElementBuilder::validateEmbedding(
    const Mesh& cellAlignedSurface,
    const Components& components,
    bool allowPerimeter)
{
    const std::map<Cell, CoordinateId> cellToId =
        buildCellToId(cellAlignedSurface);
    const std::map<EdgeKey, std::size_t> faceCounts =
        buildEdgeFaceCounts(cellAlignedSurface);

    for (const auto& component : components) {
        for (const Segment& segment : component) {
            const auto first = cellToId.find(segment[0]);
            const auto second = cellToId.find(segment[1]);
            if (first == cellToId.end() || second == cellToId.end()) {
                throw std::runtime_error(
                    "Embedded line segment is not embedded in any surface: " +
                    segmentToString(segment));
            }

            const auto count = faceCounts.find(edgeKey(first->second, second->second));
            if (count == faceCounts.end() || count->second == 0) {
                throw std::runtime_error(
                    "Embedded line segment is not embedded in any surface: " +
                    segmentToString(segment));
            }
            if (count->second <= 1 && !allowPerimeter) {
                throw std::runtime_error(
                    "Embedded line segment runs over the perimeter of a surface: " +
                    segmentToString(segment));
            }
        }
    }
}

Mesh EmbeddedElementBuilder::buildSurfaceStrip(
    const Mesh& cellAlignedSurface,
    const Components& components)
{
    const std::map<Cell, CoordinateId> cellToId =
        buildCellToId(cellAlignedSurface);

    std::vector<Element> quads;
    std::map<EdgeKey, std::vector<ElementId>> edgeQuads;
    for (ElementId elementId = 0;
         elementId < cellAlignedSurface.groups[0].elements.size();
         ++elementId) {
        const Element& element = cellAlignedSurface.groups[0].elements[elementId];
        if (!element.isQuad()) {
            continue;
        }
        const ElementId quadId = quads.size();
        quads.push_back(element);
        for (std::size_t vertex = 0; vertex < 4; ++vertex) {
            edgeQuads[edgeKey(
                element.vertices[vertex],
                element.vertices[(vertex + 1) % 4])].push_back(quadId);
        }
    }

    std::vector<std::set<EdgeKey>> quadEdges;
    quadEdges.reserve(quads.size());
    for (const Element& quad : quads) {
        quadEdges.push_back(quadEdgeSet(quad));
    }

    std::map<ElementId, std::set<ElementId>> adjacency;
    for (const auto& entry : edgeQuads) {
        const std::vector<ElementId>& ids = entry.second;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            for (std::size_t j = i + 1; j < ids.size(); ++j) {
                adjacency[ids[i]].insert(ids[j]);
                adjacency[ids[j]].insert(ids[i]);
            }
        }
    }

    std::set<ElementId> selected;
    for (const auto& component : components) {
        std::vector<std::vector<ElementId>> candidates;
        candidates.reserve(component.size());
        for (const Segment& segment : component) {
            const auto first = cellToId.find(segment[0]);
            const auto second = cellToId.find(segment[1]);
            if (first == cellToId.end() || second == cellToId.end()) {
                throw std::runtime_error(
                    "Embedded line segment is not embedded in a surface quad: " +
                    segmentToString(segment));
            }
            const auto edgeIt = edgeQuads.find(edgeKey(first->second, second->second));
            if (edgeIt == edgeQuads.end() || edgeIt->second.empty()) {
                throw std::runtime_error(
                    "Embedded line segment is not embedded in a surface quad: " +
                    segmentToString(segment));
            }
            std::vector<ElementId> unique = edgeIt->second;
            // Equal-cost sides are broken by host element id. This keeps the
            // result deterministic for a given host mesh while avoiding an
            // arbitrary geometric preference between two valid sides.
            std::sort(unique.begin(), unique.end());
            unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
            candidates.push_back(std::move(unique));
        }

        const std::vector<ElementId> sequence = solveComponent(
            component, candidates, quads, quadEdges, adjacency, cellToId);
        selected.insert(sequence.begin(), sequence.end());
    }

    if (selected.empty()) {
        throw std::runtime_error("Embedded surface strip is empty.");
    }

    std::vector<EdgeKey> segmentEdges;
    for (const auto& component : components) {
        for (const Segment& segment : component) {
            const auto first = cellToId.find(segment[0]);
            const auto second = cellToId.find(segment[1]);
            if (first == cellToId.end() || second == cellToId.end()) {
                continue;
            }
            segmentEdges.push_back(edgeKey(first->second, second->second));
        }
    }

    std::vector<std::vector<ElementId>> coveringQuads(segmentEdges.size());
    std::map<ElementId, std::vector<std::size_t>> coveredSegments;
    for (std::size_t segmentId = 0;
         segmentId < segmentEdges.size();
         ++segmentId) {
        const auto quadsForEdge = edgeQuads.find(segmentEdges[segmentId]);
        if (quadsForEdge == edgeQuads.end()) {
            continue;
        }
        for (const ElementId quadId : quadsForEdge->second) {
            if (selected.count(quadId) == 0) {
                continue;
            }
            coveringQuads[segmentId].push_back(quadId);
            coveredSegments[quadId].push_back(segmentId);
        }
    }

    auto isConnectedSelection = [&adjacency](
        const std::set<ElementId>& selection) {
        if (selection.empty()) {
            return false;
        }
        std::set<ElementId> visited;
        std::queue<ElementId> pending;
        pending.push(*selection.begin());
        while (!pending.empty()) {
            const ElementId current = pending.front();
            pending.pop();
            if (!visited.insert(current).second) {
                continue;
            }
            const auto neighbours = adjacency.find(current);
            if (neighbours == adjacency.end()) {
                continue;
            }
            for (const ElementId neighbour : neighbours->second) {
                if (selection.count(neighbour) != 0) {
                    pending.push(neighbour);
                }
            }
        }
        return visited.size() == selection.size();
    };

    auto buildSelectedAdjacency = [&selected, &adjacency]() {
        std::map<ElementId, std::set<ElementId>> result;
        for (const ElementId quadId : selected) {
            const auto neighbours = adjacency.find(quadId);
            if (neighbours == adjacency.end()) {
                continue;
            }
            for (const ElementId neighbour : neighbours->second) {
                if (selected.count(neighbour) != 0) {
                    result[quadId].insert(neighbour);
                }
            }
        }
        return result;
    };

    auto canRemove = [&](const ElementId quadId) {
        const auto covered = coveredSegments.find(quadId);
        if (covered != coveredSegments.end()) {
            for (const std::size_t segmentId : covered->second) {
                if (coveringQuads[segmentId].size() <= 1) {
                    return false;
                }
            }
        }
        selected.erase(quadId);
        const bool connected = isConnectedSelection(selected);
        if (!connected) {
            selected.insert(quadId);
        }
        return connected;
    };

    auto removeQuad = [&](const ElementId quadId) {
        selected.erase(quadId);
        const auto covered = coveredSegments.find(quadId);
        if (covered == coveredSegments.end()) {
            return;
        }
        for (const std::size_t segmentId : covered->second) {
            std::vector<ElementId>& covering = coveringQuads[segmentId];
            covering.erase(
                std::remove(covering.begin(), covering.end(), quadId),
                covering.end());
        }
    };

    // Connector insertion can leave side quads that are adjacent to the
    // selected strip but not needed to cover any line segment. They turn a
    // one-quad-wide strip into a branching selection, so prune them only
    // around vertices/quads that are already problematic.
    bool removed = true;
    while (removed) {
        removed = false;
        const auto currentAdjacency = buildSelectedAdjacency();

        std::set<ElementId> problematic;
        for (const ElementId quadId : selected) {
            const auto neighbours = currentAdjacency.find(quadId);
            if (neighbours != currentAdjacency.end()
                && neighbours->second.size() > 2) {
                problematic.insert(quadId);
            }
        }

        std::map<CoordinateId, std::vector<ElementId>> quadsAtVertex;
        for (const ElementId quadId : selected) {
            for (const CoordinateId vertex : quads[quadId].vertices) {
                quadsAtVertex[vertex].push_back(quadId);
            }
        }
        for (const auto& entry : quadsAtVertex) {
            const std::vector<ElementId>& incident = entry.second;
            if (incident.size() < 4) {
                continue;
            }
            std::size_t links = 0;
            for (std::size_t i = 0; i < incident.size(); ++i) {
                const auto neighbours = currentAdjacency.find(incident[i]);
                if (neighbours == currentAdjacency.end()) {
                    continue;
                }
                for (std::size_t j = i + 1; j < incident.size(); ++j) {
                    if (neighbours->second.count(incident[j]) != 0) {
                        ++links;
                    }
                }
            }
            if (links != incident.size() - 1) {
                problematic.insert(incident.begin(), incident.end());
            }
        }

        for (const ElementId quadId : problematic) {
            std::vector<ElementId> candidates{ quadId };
            const auto neighbours = currentAdjacency.find(quadId);
            if (neighbours != currentAdjacency.end()) {
                candidates.insert(
                    candidates.end(),
                    neighbours->second.begin(),
                    neighbours->second.end());
            }
            for (const ElementId candidate : candidates) {
                if (selected.count(candidate) == 0 || !canRemove(candidate)) {
                    continue;
                }
                removeQuad(candidate);
                removed = true;
                break;
            }
            if (removed) {
                break;
            }
        }
    }

    std::map<std::array<CoordinateId, 4>, ElementId> uniqueSelected;
    std::set<ElementId> deduplicated;
    for (const ElementId quadId : selected) {
        std::array<CoordinateId, 4> key;
        for (std::size_t vertex = 0; vertex < 4; ++vertex) {
            key[vertex] = quads[quadId].vertices[vertex];
        }
        std::sort(key.begin(), key.end());
        if (uniqueSelected.emplace(key, quadId).second) {
            deduplicated.insert(quadId);
        }
    }

    std::map<ElementId, std::set<ElementId>> selectedAdjacency;
    for (const ElementId quadId : deduplicated) {
        for (const ElementId neighbour : adjacency[quadId]) {
            if (deduplicated.count(neighbour) != 0) {
                selectedAdjacency[quadId].insert(neighbour);
            }
        }
    }

    std::set<ElementId> visited;
    std::queue<ElementId> pending;
    pending.push(*deduplicated.begin());
    while (!pending.empty()) {
        const ElementId current = pending.front();
        pending.pop();
        if (!visited.insert(current).second) {
            continue;
        }
        for (const ElementId neighbour : selectedAdjacency[current]) {
            pending.push(neighbour);
        }
    }
    if (visited.size() != deduplicated.size()) {
        throw std::runtime_error(
            "Embedded surface strip is not continuous.");
    }

    std::map<CoordinateId, std::vector<ElementId>> quadsAtVertex;
    for (const ElementId quadId : deduplicated) {
        for (const CoordinateId vertex : quads[quadId].vertices) {
            quadsAtVertex[vertex].push_back(quadId);
        }
    }
    for (const auto& entry : quadsAtVertex) {
        const std::vector<ElementId>& incident = entry.second;
        if (incident.size() < 4) {
            continue;
        }

        std::size_t links = 0;
        for (std::size_t i = 0; i < incident.size(); ++i) {
            const auto neighbours = selectedAdjacency.find(incident[i]);
            for (std::size_t j = i + 1; j < incident.size(); ++j) {
                if (neighbours != selectedAdjacency.end()
                    && neighbours->second.count(incident[j]) != 0) {
                    ++links;
                }
            }
        }

        // A 2x2 block has four quads and four edge links around its centre.
        // A one-quad-wide strip may also wrap around a convex corner with
        // four quads at one vertex, but then its link graph is a path.
        if (links != incident.size() - 1) {
            throw std::runtime_error(
                "Embedded surface contains a four-quad group; only strips "
                "of surface quads are allowed.");
        }
    }
    // A one-quad-wide strip is normally a path or a closed loop: every
    // selected quad has at most two edge-neighbours. At a staircase saddle
    // the strip can fold back onto itself, giving one quad three neighbours.
    // Allow that only when two of those neighbours are edge-adjacent, which
    // identifies a local fold rather than a T-shaped branch.
    for (const ElementId quadId : deduplicated) {
        const std::set<ElementId>& neighbours = selectedAdjacency[quadId];
        if (neighbours.size() <= 2) {
            continue;
        }
        if (neighbours.size() != 3) {
            throw std::runtime_error(
                "Embedded surface strip branches or is wider than one quad.");
        }

        const std::vector<ElementId> neighbourList(
            neighbours.begin(), neighbours.end());
        bool fold = false;
        for (std::size_t i = 0; i < neighbourList.size() && !fold; ++i) {
            const auto neighbourNeighbours =
                selectedAdjacency.find(neighbourList[i]);
            if (neighbourNeighbours == selectedAdjacency.end()) {
                continue;
            }
            for (std::size_t j = i + 1; j < neighbourList.size(); ++j) {
                if (neighbourNeighbours->second.count(neighbourList[j]) != 0) {
                    fold = true;
                    break;
                }
            }
        }
        if (!fold) {
            throw std::runtime_error(
                "Embedded surface strip branches or is wider than one quad.");
        }
    }

    Mesh result = cellAlignedSurface;
    result.groups.resize(1);
    result.groups[0].elements.clear();
    result.groups[0].name = "embeddedSurface";
    for (const ElementId quadId : deduplicated) {
        result.groups[0].elements.push_back(quads[quadId]);
    }
    utils::RedundancyCleaner::cleanCoords(result);
    return result;
}

}
}
