#include "Structurer.h"

#include "utils/Cleaner.h"

namespace meshlib {
namespace tessellator {
using namespace utils;

Structurer::Structurer(const Mesh& inputMesh) : GridTools(inputMesh.grid)
{
    mesh_.grid = inputMesh.grid;

    mesh_.coordinates.reserve(inputMesh.coordinates.size() * 2);

    mesh_.groups.resize(inputMesh.groups.size());

    for (std::size_t g = 0; g < mesh_.groups.size(); ++g) {

        auto& inputGroup = inputMesh.groups[g];
        auto& meshGroup = mesh_.groups[g];
        meshGroup.elements.reserve(inputGroup.elements.size() * 2);

        for (auto & element : inputGroup.elements) {
            if (element.isLine()) {
                this->processLineAndAddToGroup(element, inputMesh.coordinates, mesh_.coordinates, meshGroup);
            }
            else if (element.isTriangle()) {
                this->processTriangleAndAddToGroup(element, inputMesh.coordinates, meshGroup);
            }
        }
    }

    Cleaner::fuseCoords(mesh_);
    Cleaner::cleanCoords(mesh_);
}


void Structurer::processTriangleAndAddToGroup(const Element& triangle, const Coordinates& originalRelativeCoordinates, Group& group){
    Group edges;

    edges.elements = {
        Element({triangle.vertices[0], triangle.vertices[1]}),
        Element({triangle.vertices[1], triangle.vertices[2]}),
        Element({triangle.vertices[2], triangle.vertices[0]}),
    };

    Mesh auxiliarMesh;
    auxiliarMesh.grid = this->mesh_.grid;
    auxiliarMesh.groups = { Group() };
    Group& processedEdges = auxiliarMesh.groups[0];
    auxiliarMesh.groups[0].elements.reserve(9);

    for (auto& edge : edges.elements) {
        this->processLineAndAddToGroup(edge, originalRelativeCoordinates, auxiliarMesh.coordinates, processedEdges);
    }

    Cleaner::fuseCoords(auxiliarMesh);
    Cleaner::cleanCoords(auxiliarMesh);

    CoordinateId newCoordinateId = this->mesh_.coordinates.size();

    mesh_.coordinates.insert(mesh_.coordinates.end(), auxiliarMesh.coordinates.begin(), auxiliarMesh.coordinates.end());

    std::map<Surfel, IdSet> coordinateIdsByCellSurface;

    calculateCoordinateIdsByCellSurface(auxiliarMesh.coordinates, coordinateIdsByCellSurface);

    auto cellSurfaceIt = coordinateIdsByCellSurface.begin();
    while (cellSurfaceIt != coordinateIdsByCellSurface.end()) {
        if (cellSurfaceIt->second.size() != 4) {
            cellSurfaceIt = coordinateIdsByCellSurface.erase(cellSurfaceIt);
        }
        else {
            ++cellSurfaceIt;
        }
    }
    bool hasFirstSurface = false;
    bool hasSecondSurface = false;
    IdSet* firstCellSurfaceIds;
    IdSet* secondCellSurfaceIds;

    if (coordinateIdsByCellSurface.size() == 2) {
        hasFirstSurface = true;
        hasSecondSurface = true;
        firstCellSurfaceIds = &coordinateIdsByCellSurface.begin()->second;
        secondCellSurfaceIds = &coordinateIdsByCellSurface.rbegin()->second;
        auto& firstIt = firstCellSurfaceIds->begin();
        auto& secondIt = secondCellSurfaceIds->begin();
        for (firstIt; firstIt != firstCellSurfaceIds->end() && secondIt != firstCellSurfaceIds->end(); ++firstIt, ++secondIt) {
            if (*firstIt < *secondIt) {
                break;
            }
            if (*secondIt < *firstIt) {
                std::swap(firstCellSurfaceIds, secondCellSurfaceIds);
                break;
            }
        }
    }
    else if (coordinateIdsByCellSurface.size() == 1) {
        auto& surfaceCoordinateIds = coordinateIdsByCellSurface.begin()->second;
        if (*surfaceCoordinateIds.begin() == 0) {
            hasFirstSurface = true;
            firstCellSurfaceIds = &surfaceCoordinateIds;
        }
        else {
            hasSecondSurface = true;
            secondCellSurfaceIds = &surfaceCoordinateIds;
        }
    }

    auto newElementsStartingPosition = group.elements.size();

    if (hasFirstSurface) {
        Element surface({}, Element::Type::Surface);
        surface.vertices.insert(surface.vertices.begin(), firstCellSurfaceIds->begin(), firstCellSurfaceIds->end());
        group.elements.push_back(surface);
    }

    std::size_t e = 0;
    if (coordinateIdsByCellSurface.size() != 2) {
        while (hasFirstSurface && e < processedEdges.elements.size() && isEdgePartOfCellSurface(processedEdges.elements[e], *firstCellSurfaceIds)) {
            ++e;
        }

        while (e < processedEdges.elements.size() 
            && (!hasFirstSurface || !isEdgePartOfCellSurface(processedEdges.elements[e], *firstCellSurfaceIds))
            && (!hasSecondSurface || !isEdgePartOfCellSurface(processedEdges.elements[e], *secondCellSurfaceIds))) {
            group.elements.push_back(processedEdges.elements[e]);
            ++e;
        }
    }

    if (hasSecondSurface) {
        Element surface({}, Element::Type::Surface);
        surface.vertices.insert(surface.vertices.begin(), secondCellSurfaceIds->begin(), secondCellSurfaceIds->end());
        group.elements.push_back(surface);
    }

    if (coordinateIdsByCellSurface.size() != 2) {
        while (hasSecondSurface && e < processedEdges.elements.size() && isEdgePartOfCellSurface(processedEdges.elements[e], *secondCellSurfaceIds)) {
            ++e;
        }

        while (e < processedEdges.elements.size()
            && (!hasFirstSurface || !isEdgePartOfCellSurface(processedEdges.elements[e], *firstCellSurfaceIds))) {
            group.elements.push_back(processedEdges.elements[e]);
            ++e;
        }
    }

    for (std::size_t e = newElementsStartingPosition; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        for (std::size_t v = 0; v < element.vertices.size(); ++v) {
            element.vertices[v] += newCoordinateId;
        }
    }
}

void Structurer::processLineAndAddToGroup(const Element& line, const Coordinates& originalRelativeCoordinates, Coordinates& resultCoordinates, Group& group) {
    auto startRelative = originalRelativeCoordinates[line.vertices[0]];
    auto endRelative = originalRelativeCoordinates[line.vertices[1]];

    CoordinateId startIndex = resultCoordinates.size();

    std::vector<Cell> cells = calculateMiddleCellsBetweenTwoCoordinates(startRelative, endRelative);

    auto startRelativePosition = this->toRelative(cells.front());
    resultCoordinates.push_back(startRelativePosition);

    if (cells.size() == 1) {
        group.elements.push_back(Element({ startIndex }, Element::Type::Node));
        return;
    }

    for (std::size_t v = 1; v < cells.size(); ++v) {
        auto endRelativePosition = this->toRelative(cells[v]);
        CoordinateId endIndex = startIndex + 1;
        resultCoordinates.push_back(endRelativePosition);

        group.elements.push_back(Element({ startIndex, endIndex }, Element::Type::Line));

        std::swap(startRelativePosition, endRelativePosition);
        ++startIndex;
    }
}

std:: vector<Cell> Structurer::calculateMiddleCellsBetweenTwoCoordinates(Coordinate& startExtreme, Coordinate& endExtreme) {
    // TODO: Use references instead of copying cordinates for method parameters

    auto startCell = this->calculateStructuredCell(startExtreme);
    auto endCell = this->calculateStructuredCell(endExtreme);
    auto startStructured = this->toRelative(startCell);
    auto endStructured = this->toRelative(endCell);

    std::vector<Cell> cells;
    cells.reserve(4);
    cells.push_back(startCell);
    

    Coordinate centerVector = startStructured + (endStructured - startStructured) / 2.0;
    Coordinate distanceVector = endExtreme - startExtreme;
    Coordinate scaleVector;

    std::map<double, Coordinate> sortedIntersections;

    std::vector<Axis> differentAxes = calculateDifferentAxesBetweenCells(startCell, endCell);

    if (differentAxes.size() > 1) {
        for (Axis scaleAxis : differentAxes) {
            scaleVector[scaleAxis] = distanceVector[scaleAxis] / (centerVector[scaleAxis] - startExtreme[scaleAxis]);
            Coordinate componentPoint;

            for (Axis intersectionAxis = X; intersectionAxis <= Z; ++intersectionAxis) {
                if (intersectionAxis == scaleAxis) {
                    componentPoint[intersectionAxis] = centerVector[scaleAxis];
                }
                else {
                    componentPoint[intersectionAxis] = startExtreme[intersectionAxis] + distanceVector[intersectionAxis] / scaleVector[scaleAxis];
                }
            }

            double componentDistance = (componentPoint - startExtreme).norm();
            sortedIntersections[componentDistance] = componentPoint;
        }
        auto intersectionIt = sortedIntersections.begin();
        auto nextIntersectionIt = sortedIntersections.begin();
        ++nextIntersectionIt;
        while (intersectionIt != sortedIntersections.end()) {
            auto& point = intersectionIt->second;

            std::set<Axis> equalAxes;

            for (Axis axis = X; axis <= Z; ++axis) {
                Axis secondAxis = (axis + 1) % 3;

                if (!approxDir(centerVector[axis], 0.0) && !approxDir(centerVector[axis] - 1.0, 0.0)
                    && !approxDir(centerVector[secondAxis], 0.0) && !approxDir(centerVector[secondAxis] - 1.0, 0.0)
                    && approxDir(centerVector[axis] - point[axis], 0.0) && approxDir(centerVector[secondAxis] - point[secondAxis], 0.0)) {
                    equalAxes.insert(axis);
                    equalAxes.insert(secondAxis);
                }
            }

            if (equalAxes.size() != 0) {
                Cell firstMiddleCell = cells.back();
                auto forcedAxisIt = equalAxes.begin();

                firstMiddleCell[*forcedAxisIt] = endCell[*forcedAxisIt];
                ++forcedAxisIt;
                Cell secondMiddleCell = firstMiddleCell;
                secondMiddleCell[*forcedAxisIt] = endCell[*forcedAxisIt];

                cells.push_back(firstMiddleCell);
                cells.push_back(secondMiddleCell);
            }
            else if (nextIntersectionIt != sortedIntersections.end()) {
                Coordinate& nextPoint = nextIntersectionIt->second;

                Coordinate middlePoint = (point + nextPoint) / 2;
                Cell middleCell = calculateStructuredCell(middlePoint);
                cells.push_back(middleCell);
            }

            ++intersectionIt;
            if (nextIntersectionIt != sortedIntersections.end()) {
                ++nextIntersectionIt;
            }
        }
    }

    if (cells.back() != endCell) {
        cells.push_back(endCell);
    }

    return cells;
}

Cell Structurer::calculateStructuredCell(const Coordinate& relativeCoordinate) const
{
    auto resultCell = this->toCell(relativeCoordinate);

    for (std::size_t axis = 0; axis < 3; ++axis) {
        auto distance = relativeCoordinate[axis] - resultCell[axis];

        if (distance >= 0.5) {
            ++resultCell[axis];
        }
    }

    return resultCell;
}

std::size_t Structurer::calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            ++difference;
        }
    }
    return difference;

}

std::vector<Axis> Structurer::calculateDifferentAxesBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    std::vector<Axis> difference;
    difference.reserve(3);

    for (Axis axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            difference.push_back(axis);
        }
    }
    return difference;
}

std::vector<Axis> Structurer::calculateEqualAxesBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    std::vector<Axis> equalAxes;
    equalAxes.reserve(3);

    for (Axis axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] == secondCell[axis]) {
            equalAxes.push_back(axis);
        }
    }
    return equalAxes;
}

void Structurer::calculateCoordinateIdsByCellSurface(const Coordinates& coordinates, std::map<Surfel, IdSet>& coordinatesByCellSurface) {
    Cell minCell({
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max()
        });

    Cell maxCell({
        std::numeric_limits<CellDir>::lowest(),
        std::numeric_limits<CellDir>::lowest(),
        std::numeric_limits<CellDir>::lowest()
        });

    for (auto & coordinate : coordinates) {
        for (Axis axis = X; axis < 3; ++axis) {
            minCell[axis] = std::min(minCell[axis], toCellDir(coordinate[axis]));
            maxCell[axis] = std::max(maxCell[axis], toCellDir(coordinate[axis]));
        }
    }

    auto equalAxesBetweenExtremes = calculateEqualAxesBetweenCells(minCell, maxCell);
    for (auto axis : equalAxesBetweenExtremes) {
        ++maxCell[axis];
    }

    for (CoordinateId v = 0; v < coordinates.size(); ++v) {
        Cell coordinateCell = toCell(coordinates[v]);
        auto minCellEqualAxes = calculateEqualAxesBetweenCells(minCell, coordinateCell);
        auto maxCellEqualAxes = calculateEqualAxesBetweenCells(maxCell, coordinateCell);

        for (auto axis : minCellEqualAxes) {
            coordinatesByCellSurface[{minCell, axis}].insert(v);
        }

        for (auto axis : maxCellEqualAxes) {
            coordinatesByCellSurface[{maxCell, axis}].insert(v);
        }
    }
}

bool Structurer::isEdgePartOfCellSurface(const Element& edge, const IdSet& surfaceCoordinateIds) const {
    if (!edge.isLine() || surfaceCoordinateIds.size() != 4) {
        return false;
    }

    for (auto vertex : edge.vertices) {
        if (surfaceCoordinateIds.find(vertex) == surfaceCoordinateIds.end()) {
            return false;
        }
    }

    return true;
}


}
}