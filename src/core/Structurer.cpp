#include "Structurer.h"

#include "utils/Cleaner.h"

namespace meshlib {
namespace core {
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
        Element({triangle.vertices[0], triangle.vertices[1]}, Element::Type::Line),
        Element({triangle.vertices[1], triangle.vertices[2]}, Element::Type::Line),
        Element({triangle.vertices[2], triangle.vertices[0]}, Element::Type::Line),
    };

    Mesh auxiliarMesh;
    auxiliarMesh.grid = this->mesh_.grid;
    auxiliarMesh.groups = { Group() };
    Group& processedEdges = auxiliarMesh.groups[0];
    auxiliarMesh.groups[0].elements.reserve(9);
    int pureDiagonalIndex = -1;

    for (std::size_t index = 0; index < edges.elements.size(); ++index) {
        auto& edge = edges.elements[index];
        if (isPureDiagonal(edge, originalRelativeCoordinates)) {
            pureDiagonalIndex = int(index);
        }
        else {
            this->processLineAndAddToGroup(edge, originalRelativeCoordinates, auxiliarMesh.coordinates, processedEdges);
        }
    }

    Cleaner::fuseCoords(auxiliarMesh);
    Cleaner::cleanCoords(auxiliarMesh);

    std::map<Surfel, IdSet> idSetByCellSurface;
    std::map<Surfel, CoordinateIds> coordinateIdsByCellSurface;

    calculateCoordinateIdSetByCellSurface(auxiliarMesh.coordinates, idSetByCellSurface);

    filterSurfacesFromCoordinateIds(
        triangle.vertices,
        pureDiagonalIndex,
        originalRelativeCoordinates,
        idSetByCellSurface,
        auxiliarMesh.coordinates,
        coordinateIdsByCellSurface);

    if(auxiliarMesh.coordinates.size() == 6 && processedEdges.elements.size() == 6 && coordinateIdsByCellSurface.size() == 0){
        processedEdges.elements.clear();
        coordinateIdsByCellSurface.clear();
        idSetByCellSurface.clear();

        for (std::size_t index = 0; index < edges.elements.size(); ++index) {
            auto& edge = edges.elements[index];
            if (index != pureDiagonalIndex) {
                this->processLineAndAddToGroup(edge, originalRelativeCoordinates, auxiliarMesh.coordinates, processedEdges, true);
            }
        }

        Cleaner::fuseCoords(auxiliarMesh);
        Cleaner::cleanCoords(auxiliarMesh);
        
        calculateCoordinateIdSetByCellSurface(auxiliarMesh.coordinates, idSetByCellSurface);

        filterSurfacesFromCoordinateIds(
            triangle.vertices,
            pureDiagonalIndex,
            originalRelativeCoordinates,
            idSetByCellSurface,
            auxiliarMesh.coordinates,
            coordinateIdsByCellSurface);
        }

    CoordinateId newCoordinateId = this->mesh_.coordinates.size();

    mesh_.coordinates.insert(mesh_.coordinates.end(), auxiliarMesh.coordinates.begin(), auxiliarMesh.coordinates.end());

    std::vector<bool> surfacePresenceList = {false, false};
    std::vector<Surfel> cellSurfacePlanes;
    std::vector<CoordinateIds*> cellSurfaceIdsList;

    if (coordinateIdsByCellSurface.size() >= 2) {
        surfacePresenceList.clear();
        surfacePresenceList.reserve(3);
        cellSurfacePlanes.reserve(3);
        cellSurfaceIdsList.reserve(3);

        for(auto it = coordinateIdsByCellSurface.begin(); it != coordinateIdsByCellSurface.end(); ++it){
            surfacePresenceList.push_back(true);
            cellSurfacePlanes.push_back(it->first);
            cellSurfaceIdsList.push_back(&it->second);
        }

        for(std::size_t planeIndex = 0; planeIndex < (cellSurfacePlanes.size() - 1); ++planeIndex){
            auto& firstCellSurfacePlane = cellSurfacePlanes[planeIndex];
            auto& secondCellSurfacePlane = cellSurfacePlanes[planeIndex+1];
            auto& firstCellSurfaceIds = cellSurfaceIdsList[planeIndex];
            auto& secondCellSurfaceIds = cellSurfaceIdsList[planeIndex + 1];

            for (std::size_t index = 0; index != firstCellSurfaceIds->size(); ++index) {
                if (firstCellSurfaceIds[index] < secondCellSurfaceIds[index]) {
                    break;
                }
                if (secondCellSurfaceIds[index] < firstCellSurfaceIds[index]) {
                    std::swap(firstCellSurfaceIds, secondCellSurfaceIds);
                    std::swap(firstCellSurfacePlane, secondCellSurfacePlane);
                    break;
                }
            }
        }
        
        
    }
    else if (coordinateIdsByCellSurface.size() == 1) {
        auto surfaceCoordinateIt = coordinateIdsByCellSurface.begin();
        cellSurfacePlanes.resize(2);
        cellSurfaceIdsList.resize(2);

        if (surfaceCoordinateIt->second[0] == 0 || pureDiagonalIndex == 0) {
            surfacePresenceList[0] = true;
            cellSurfacePlanes[0] = surfaceCoordinateIt->first;
            cellSurfaceIdsList[0] = &surfaceCoordinateIt->second;
        }
        else {
            surfacePresenceList[1] = true;
            cellSurfacePlanes[1] = surfaceCoordinateIt->first;
            cellSurfaceIdsList[1] = &surfaceCoordinateIt->second;
        }
    }

    auto newElementsStartingPosition = group.elements.size();
    
    std::size_t e = 0;
    
    for(std::size_t surfaceIndex = 0; surfaceIndex < cellSurfacePlanes.size(); ++surfaceIndex){
        if (surfacePresenceList[surfaceIndex]) {
            Element surface({}, Element::Type::Surface);
            surface.vertices.insert(surface.vertices.begin(), cellSurfaceIdsList[surfaceIndex]->begin(), cellSurfaceIdsList[surfaceIndex]->end());
            group.elements.push_back(surface);
        }
    }
    if(coordinateIdsByCellSurface.size() < 2){
        for(std::size_t e = 0; e < processedEdges.elements.size(); ++e){
            bool isInSurface = false;
            for (std::size_t surfaceIndex = 0; surfaceIndex < cellSurfacePlanes.size(); ++surfaceIndex){
                isInSurface = isInSurface
                    || surfacePresenceList[surfaceIndex]
                    && isEdgePartOfCellSurface(processedEdges.elements[e], *cellSurfaceIdsList[surfaceIndex]);
            }
            if (!isInSurface){
                group.elements.push_back(processedEdges.elements[e]);
            }
        }
    }

    for (std::size_t e = newElementsStartingPosition; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        for (std::size_t v = 0; v < element.vertices.size(); ++v) {
            element.vertices[v] += newCoordinateId;
        }
    }
}

bool Structurer::isRelativeInCellsVector(const Relative& relative, const std::vector<Cell> & cells) const {
    Cell convertedCell = toCell(relative);

    for (auto& listCell : cells) {
        if (listCell == convertedCell) {
            return true;
        }
    }

    return false;
}

void Structurer::filterSurfacesFromCoordinateIds(
    const CoordinateIds& triangleVertices,
    int pureDiagonalIndex,
    const Coordinates& originalRelativeCoordinates,
    const std::map<Surfel, IdSet>& idSetByCellSurface,
    Coordinates& structuredCoordinates,
    std::map<Surfel, CoordinateIds> & coordinateIdsByCellSurface
) {
    std::vector<Cell> structuredOriginalVertexCells;
    structuredOriginalVertexCells.reserve(3);
    for (CoordinateId v : triangleVertices) {
        structuredOriginalVertexCells.push_back(calculateStructuredCell(originalRelativeCoordinates[v]));
    }

    auto cellSurfaceIt = idSetByCellSurface.begin();
    while (cellSurfaceIt != idSetByCellSurface.end()) {
        auto& plane = cellSurfaceIt->first;
        auto& idSet = cellSurfaceIt->second;
        auto numberOfSurfacePoints = idSet.size();


        std::vector<Cell> projectedCells;
        bool isCorrectSurface = false;

        if (pureDiagonalIndex >= 0 && numberOfSurfacePoints == 3) {
            for (auto& cell : structuredOriginalVertexCells) {
                projectedCells.push_back(cell);
                projectedCells.back()[plane.second] = plane.first[plane.second];
            }

            isCorrectSurface = true;
            for (auto vertexIdIterator = idSet.begin(); isCorrectSurface && vertexIdIterator != idSet.end(); ++vertexIdIterator) {
                auto& surfaceCoordinate = structuredCoordinates[*vertexIdIterator];
                isCorrectSurface = isRelativeInCellsVector(surfaceCoordinate, projectedCells);
            }
        }

        if (numberOfSurfacePoints == 4 || (pureDiagonalIndex >= 0 && isCorrectSurface)) {
            coordinateIdsByCellSurface[plane] = CoordinateIds({});
            coordinateIdsByCellSurface[plane].insert(coordinateIdsByCellSurface[plane].begin(), idSet.begin(), idSet.end());
        }

        if (pureDiagonalIndex >= 0 && isCorrectSurface) {
            auto& surfaceIds = coordinateIdsByCellSurface[plane];
            Cell missingCell = projectedCells[pureDiagonalIndex];
            for (auto coordinateId : surfaceIds) {
                Cell surfaceCell = toCell(structuredCoordinates[coordinateId]);
                auto differentAxes = calculateDifferentAxesBetweenCells(projectedCells[pureDiagonalIndex], surfaceCell);
                if (differentAxes.size() == 1) {
                    Axis axisToChange = X;

                    while (axisToChange == plane.second || axisToChange == differentAxes[0]) {
                        ++axisToChange;
                    }

                    missingCell[axisToChange] = projectedCells[(pureDiagonalIndex + 1) % 3][axisToChange];
                    break;
                }
            }
            for (auto coordinateIt = surfaceIds.begin(); coordinateIt != surfaceIds.end(); ++coordinateIt) {
                Coordinate surfaceCoordinate = structuredCoordinates[*coordinateIt];

                if (projectedCells[pureDiagonalIndex] == toCell(surfaceCoordinate)) {
                    auto positionToInsert = coordinateIt + 1;
                    CoordinateId missingCoordinateId = structuredCoordinates.size();
                    structuredCoordinates.push_back(toRelative(missingCell));
                    surfaceIds.insert(positionToInsert, missingCoordinateId);
                    break;
                }
            }
        }

        ++cellSurfaceIt;
    }
}

void Structurer::processLineAndAddToGroup(const Element& line, const Coordinates& originalRelativeCoordinates, Coordinates& resultCoordinates, Group& group, bool invertPriority) {
    auto startRelative = originalRelativeCoordinates[line.vertices[0]];
    auto endRelative = originalRelativeCoordinates[line.vertices[1]];

    CoordinateId startIndex = resultCoordinates.size();

    std::vector<Cell> cells = calculateMiddleCellsBetweenTwoCoordinates(startRelative, endRelative, invertPriority);

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

bool compare(float first, float second, bool useLessThan){
    if(useLessThan){
        return first < second;
    }
    return first > second;
}

std:: vector<Cell> Structurer::calculateMiddleCellsBetweenTwoCoordinates(Coordinate& startExtreme, Coordinate& endExtreme, bool invertPriority) {
    // TODO: Compare with integers as substitute for floating point numbers with three decimals.

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

            for (Axis currentAxis = X; currentAxis <= Z; ++currentAxis) {
                Axis firstAxis = currentAxis;
                Axis secondAxis = (firstAxis + 1) % 3;

                if (!approxDir(centerVector[firstAxis], startStructured[firstAxis]) && !approxDir(centerVector[firstAxis], endStructured[firstAxis])
                    && !approxDir(centerVector[secondAxis], startStructured[secondAxis]) && !approxDir(centerVector[secondAxis], endStructured[secondAxis])
                    && approxDir(centerVector[firstAxis], point[firstAxis]) && approxDir(centerVector[secondAxis], point[secondAxis])) {

                    if (compare(endExtreme[firstAxis], startExtreme[firstAxis], !invertPriority)) {
                        firstAxis = 5 - firstAxis;
                    }
                    if (compare(endExtreme[secondAxis], startExtreme[secondAxis], !invertPriority)) {
                        secondAxis = 5 - secondAxis;
                    }

                    equalAxes.insert(firstAxis);
                    equalAxes.insert(secondAxis);
                }
            }

            if (equalAxes.size() != 0) {
                Cell firstMiddleCell = cells.back();
                auto forcedAxisIt = equalAxes.begin();

                auto forcedAxis = *forcedAxisIt;

                if (forcedAxis > Z) {
                    forcedAxis = Z - (forcedAxis - 3);
                }

                firstMiddleCell[forcedAxis] = endCell[forcedAxis];
                ++forcedAxisIt;


                forcedAxis = *forcedAxisIt;

                if (forcedAxis > Z) {
                    forcedAxis = Z - (forcedAxis - 3);
                }

                Cell secondMiddleCell = firstMiddleCell;
                secondMiddleCell[forcedAxis] = endCell[forcedAxis];

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
    std::vector<Axis> differentAxes;
    differentAxes.reserve(3);

    for (Axis axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            differentAxes.push_back(axis);
        }
    }
    return differentAxes;
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

void Structurer::calculateCoordinateIdSetByCellSurface(const Coordinates& coordinates, std::map<Surfel, IdSet>& coordinatesByCellSurface) {
    Cell minCell({
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max()
        });


    for (auto & coordinate : coordinates) {
        for (Axis axis = X; axis < 3; ++axis) {
            minCell[axis] = std::min(minCell[axis], toCellDir(coordinate[axis]));
        }
    }

    Cell maxCell = minCell;

    for (auto axis = X; axis <= Z; ++axis) {
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

bool Structurer::isPureDiagonal(const Element& edge, const Coordinates & coordinates) {
    if (!edge.isLine()) {
        return false;
    }

    const auto& startPoint = coordinates[edge.vertices[0]];
    const auto& endPoint = coordinates[edge.vertices[1]];
    auto startCell = calculateStructuredCell(startPoint);
    auto endCell = calculateStructuredCell(endPoint);
    std::size_t difference = calculateDifferenceBetweenCells(startCell, endCell);

    if (difference != 3) {
        return false;
    }

    Coordinate startStructured = toRelative(startCell);
    Coordinate endStructured = toRelative(endCell);

    Coordinate centerVector = startStructured + (endStructured - startStructured) / 2.0;
    Coordinate distanceVector = endPoint - startPoint;
    Coordinate scaleVector;
    
    for (Axis axis = X; axis <= Z; ++axis) {
        if(approxDir(centerVector[axis], startPoint[axis])){
            scaleVector[axis] = 0.0;
        }
        else{
            scaleVector[axis] = distanceVector[axis] / (centerVector[axis] - startPoint[axis]);
        }
    }

    if (!approxDir(scaleVector[X], scaleVector[Y]) || !approxDir(scaleVector[Y], scaleVector[Z])) {
        return false;
    }

    return true;
}

bool Structurer::isEdgePartOfCellSurface(const Element& edge, const CoordinateIds& surfaceCoordinateIds) const {
    if (!edge.isLine()) {
        return false;
    }

    for (auto vertex : edge.vertices) {
        bool found = false;
        for (auto coordId : surfaceCoordinateIds) {
            if (vertex == coordId) {
                found = true;
                break;
            }
        }

        if (!found) {
            return false;
        }
    }

    return true;
}


}
}