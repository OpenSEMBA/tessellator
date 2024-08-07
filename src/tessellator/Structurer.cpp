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
        Relative startStructuredRelative = this->toRelative(startCell);
        Relative endStructuredRelative = this->toRelative(endCell);

        std::vector<std::size_t> differentAxes;
        differentAxes.reserve(2);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (startCell[axis] != endCell[axis]) {
                differentAxes.push_back(axis);
            }
        }

        Relative candidateOne = startStructuredRelative;
        Relative candidateTwo = startStructuredRelative;
        candidateOne[differentAxes[0]] = endStructuredRelative[differentAxes[0]];
        candidateTwo[differentAxes[1]] = endStructuredRelative[differentAxes[1]];
        Relative startToCandidateOne = candidateOne - startRelative;
        Relative startToCandidateTwo = candidateTwo - startRelative;
        Relative endToCandidateOne = candidateOne - endRelative;
        Relative endToCandidateTwo = candidateTwo - endRelative;
        
        auto startToCandidateOneMagnitude = sqrt(pow(startToCandidateOne[0], 2) + pow(startToCandidateOne[1], 2) + pow(startToCandidateOne[2], 2));
        auto startToCandidateTwoMagnitude = sqrt(pow(startToCandidateTwo[0], 2) + pow(startToCandidateTwo[1], 2) + pow(startToCandidateTwo[2], 2));
        auto endToCandidateOneMagnitude = sqrt(pow(endToCandidateOne[0], 2) + pow(endToCandidateOne[1], 2) + pow(endToCandidateOne[2], 2));
        auto endToCandidateTwoMagnitude = sqrt(pow(endToCandidateTwo[0], 2) + pow(endToCandidateTwo[1], 2) + pow(endToCandidateTwo[2], 2));

        if (approxDir(startToCandidateOneMagnitude - startToCandidateTwoMagnitude, 0.0)
            && approxDir(endToCandidateOneMagnitude - endToCandidateTwoMagnitude, 0.0)) {
            cells.push_back(this->toCell(candidateOne));
        }
        else {
            Coordinate startExtreme = startRelative;
            Coordinate endExtreme = endRelative;
            Cell middleCell;
            std::size_t tries = 0;
            do {
                if (tries > 40) {
                    throw std::logic_error("Stuck in infinite loop.");
                }
                const Coordinate middlePoint = startExtreme + (endExtreme - startExtreme) / 2.0;
                middleCell = this->calculateStructuredCell(middlePoint);

                if (middleCell == startCell) {
                    startExtreme = middlePoint;
                }
                else if (middleCell == endCell) {
                    endExtreme = middlePoint;
                }

                ++tries;

            } while (middleCell == startCell || middleCell == endCell);
            cells.push_back(middleCell);
        }
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
        else {
            Coordinate startExtreme = startRelative;
            Coordinate endExtreme = endRelative;
            Coordinate middle = startExtreme + (endExtreme - startExtreme) / 2.0;
            Coordinate thirdStep = (endRelative - startRelative) / 3.0;
            Coordinate secondPoint = startRelative + thirdStep;
            Coordinate thirdPoint = secondPoint + thirdStep;
            Cell secondCell = this->calculateStructuredCell(secondPoint);
            Cell thirdCell = this->calculateStructuredCell(thirdPoint);

            auto secondCellDifference = calculateDifferenceBetweenCells(startCell, secondCell);
            auto betweenCellsDifference = calculateDifferenceBetweenCells(secondCell, thirdCell);
            auto thirdCellDifference = calculateDifferenceBetweenCells(thirdCell, endCell);

            
            std::size_t tries = 0;
            while (secondCellDifference != 1) {
                if (tries > 200) {
                    throw std::logic_error("Stuck in infinite loop.");
                }
                ++tries;
                if (secondCellDifference == 0)
                {
                    startExtreme = secondPoint;
                }
                else if (secondCellDifference > 1) {
                    endExtreme = secondPoint;
                }
                secondPoint = startExtreme + (endExtreme - startExtreme) / 2.0;
                secondCell = this->calculateStructuredCell(secondPoint);
                secondCellDifference = calculateDifferenceBetweenCells(startCell, secondCell);
            }
            
            startExtreme = secondPoint;
            endExtreme = endRelative;
            betweenCellsDifference = calculateDifferenceBetweenCells(secondCell, thirdCell);
            thirdCellDifference = calculateDifferenceBetweenCells(thirdCell, endCell);

            tries = 0;
            while (betweenCellsDifference != 1 && thirdCellDifference != 1) {
                if (tries > 200) {
                    throw std::logic_error("Stuck in infinite loop.");
                }
                ++tries;

                if (thirdCellDifference == 0)
                {
                    endExtreme = thirdPoint;
                }
                else if (thirdCellDifference > 1) {
                    startExtreme = thirdPoint;
                }
                thirdPoint = startExtreme + (endExtreme - startExtreme) / 2.0;
                thirdCell = this->calculateStructuredCell(thirdPoint);
                betweenCellsDifference = calculateDifferenceBetweenCells(secondCell, thirdCell);
                thirdCellDifference = calculateDifferenceBetweenCells(thirdCell, endCell);
            }
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