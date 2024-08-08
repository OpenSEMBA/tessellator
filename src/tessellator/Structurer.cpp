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
                this->processLineAndAddToGroup(element, inputMesh.coordinates, meshGroup);
            }
        }
    }

    Cleaner::fuseCoords(mesh_);
    Cleaner::cleanCoords(mesh_);
}


void Structurer::processLineAndAddToGroup(const Element& line, const Coordinates& originalRelativeCoordinates, Group& group) {
    auto startRelative = originalRelativeCoordinates[line.vertices[0]];
    auto endRelative = originalRelativeCoordinates[line.vertices[1]];

    auto startCell = this->calculateStructuredCell(startRelative);
    auto endCell = this->calculateStructuredCell(endRelative);

    std::vector<Cell> cells = { startCell };

    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (startCell[axis] != endCell[axis]) {
            ++difference;
        }
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

    auto startRelativePosition = this->toRelative(cells[0]);
    CoordinateId startIndex = mesh_.coordinates.size();
    mesh_.coordinates.push_back(startRelativePosition);

    for (std::size_t v = 1; v < cells.size(); ++v) {
        auto endRelativePosition = this->toRelative(cells[v]);
        CoordinateId endIndex = startIndex + 1;
        mesh_.coordinates.push_back(endRelativePosition);

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

std::size_t Structurer::calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell){
    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            ++difference;
        }
    }
    return difference;

}


}
}