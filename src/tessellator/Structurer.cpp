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

    auto startCell = this->calculateStructuredCell(startRelative);
    auto endCell = this->calculateStructuredCell(endRelative);

    auto startRelativePosition = this->toRelative(startCell);
    CoordinateId startIndex = resultCoordinates.size();
    resultCoordinates.push_back(startRelativePosition);

    std::vector<Cell> cells = { startCell };

    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (startCell[axis] != endCell[axis]) {
            ++difference;
        }
    }
    
    if (difference == 0) {
        group.elements.push_back(Element({ startIndex, }, Element::Type::Node));
        return;
    }
    if (difference == 2) {
        auto startExtreme = startRelative;
        auto endExtreme = endRelative;
        Relative step = (endRelative - startRelative) / 2.0;
        Cell middleCell = calculateMiddleCellBetweenTwoCoordinates(startExtreme, endExtreme, step);
        cells.push_back(middleCell);
    }
    else if (difference == 3) {
        Relative startStructuredRelative = this->toRelative(startCell);
        Relative endStructuredRelative = this->toRelative(endCell);

        Coordinate startCoordinateDist = (startRelative - startStructuredRelative).abs();
        Coordinate endCoordinateDist = (endRelative - endStructuredRelative).abs();

        if (approxDir(startCoordinateDist[x] - startCoordinateDist[y], 0.0) && approxDir(startCoordinateDist[x] - startCoordinateDist[z], 0.0)
            && approxDir(endCoordinateDist[x] - endCoordinateDist[y], 0.0) && approxDir(endCoordinateDist[x] - endCoordinateDist[z], 0.0)) {
            cells.push_back(Cell({ endCell[x], startCell[y], startCell[z] }));
            cells.push_back(Cell({ endCell[x], endCell[y], startCell[z] }));
        }
        else if ((approxDir(startCoordinateDist[x] - startCoordinateDist[y], 0.0) && approxDir(endCoordinateDist[x] - endCoordinateDist[y], 0.0))
            ||   (approxDir(startCoordinateDist[x] - startCoordinateDist[z], 0.0) && approxDir(endCoordinateDist[x] - endCoordinateDist[z], 0.0))
            ||   (approxDir(startCoordinateDist[y] - startCoordinateDist[z], 0.0) && approxDir(endCoordinateDist[y] - endCoordinateDist[z], 0.0))) {
            
            std::vector<Cell> candidates({
                Cell({ endCell[x],   startCell[y], startCell[z] }),
                Cell({ endCell[x],   endCell[y],   startCell[z] }),
                Cell({ endCell[x],   startCell[y], endCell[z]   }),
                Cell({ startCell[x], endCell[y],   startCell[z] }),
                Cell({ startCell[x], endCell[y],   endCell[z]   }),
                Cell({ startCell[x], startCell[y], endCell[z]   }),
                });

            std::size_t lowestIndex = 0;
            RelativeDir lowestDistance = std::numeric_limits<RelativeDir>::max();
            
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                RelativeDir relativeDistance = (this->toRelative(candidates[i]) - startRelative).norm();
                if (relativeDistance < lowestDistance) {
                    lowestDistance = relativeDistance;
                    lowestIndex = i;
                }
            }

            Coordinate startExtreme = startRelative;
            Coordinate endExtreme = endRelative;
            Coordinate nearestPerpendicularRelative = this->toRelative(candidates[lowestIndex]);
            std::size_t position;
            if (calculateDifferenceBetweenCells(startCell, candidates[lowestIndex]) == 1) {
                startExtreme = nearestPerpendicularRelative;
                position = 1;
            }
            else {
                endExtreme = nearestPerpendicularRelative;
                position = 2;
            }

            Relative step = (endExtreme - startExtreme) / 2.0;
            Cell middleCell = calculateMiddleCellBetweenTwoCoordinates(startExtreme, endExtreme, step);
            
            if (position == 1) {
                cells.push_back(candidates[lowestIndex]);
                cells.push_back(middleCell);
            }
            else {
                cells.push_back(middleCell);
                cells.push_back(candidates[lowestIndex]);
            }
        }
        else {
            Coordinate startExtreme = startRelative;
            Coordinate endExtreme = endRelative;
            Coordinate step = (endRelative - startRelative) / 3.0;


            Cell secondCell = calculateMiddleCellBetweenTwoCoordinates(startExtreme, endExtreme, step);

            Coordinate secondPoint = startExtreme + step;

            startExtreme = secondPoint;
            endExtreme = endRelative;
            step = (endRelative - secondPoint) / 2.0;
            

            Cell thirdCell = calculateMiddleCellBetweenTwoCoordinates(startExtreme, endExtreme, step);

            cells.push_back(secondCell);
            cells.push_back(thirdCell);
        }
    }

    cells.push_back(endCell);

    for (std::size_t v = 1; v < cells.size(); ++v) {
        auto endRelativePosition = this->toRelative(cells[v]);
        CoordinateId endIndex = startIndex + 1;
        resultCoordinates.push_back(endRelativePosition);

        group.elements.push_back(Element({ startIndex, endIndex }, Element::Type::Line));

        std::swap(startRelativePosition, endRelativePosition);
        ++startIndex;
    }
}

Cell Structurer::calculateMiddleCellBetweenTwoCoordinates(Coordinate& startExtreme, Coordinate& endExtreme, Relative & step) {
    // TODO: Use references instead of copying cordinates for method parameters

    auto startCell = this->calculateStructuredCell(startExtreme);
    auto endCell = this->calculateStructuredCell(endExtreme);
    auto startStructured = this->toRelative(startCell);
    auto endStructured = this->toRelative(endCell);

    std::vector<std::size_t> differentAxes;
    differentAxes.reserve(3);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (startCell[axis] != endCell[axis]) {
            differentAxes.push_back(axis);
        }
    }


    Coordinate startCoordinateDist = (startExtreme - startStructured).abs();
    Coordinate endCoordinateDist = (endExtreme - endStructured).abs();

    if (approxDir(startCoordinateDist[differentAxes[0]] - startCoordinateDist[differentAxes[1]], 0.0)
        && approxDir(endCoordinateDist[differentAxes[0]] - endCoordinateDist[differentAxes[1]], 0.0)) {
        Relative middleStructured = startStructured;
        middleStructured[differentAxes[0]] = endStructured[differentAxes[0]];
        
        return this->toCell(middleStructured);
    }
    Cell middleCell;
    std::size_t cellDifference;

    std::size_t tries = 0;
    do {
        if (tries > 200) {
            throw std::logic_error("Stuck in infinite loop.");
        }
        const Coordinate middlePoint = startExtreme + step;
        middleCell = this->calculateStructuredCell(middlePoint);
        cellDifference = calculateDifferenceBetweenCells(startCell, middleCell);

        if (cellDifference == 0) {
            startExtreme = middlePoint;
        }
        else if (cellDifference >= 2) {
            endExtreme = middlePoint;
        }

        step = (endExtreme - startExtreme) / 2.0;

        ++tries;

    } while (cellDifference != 1);
    return middleCell;
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