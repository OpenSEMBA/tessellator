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


void Structurer::processLineAndAddToGroup(const Element& line, const Coordinates& originalCoordinates, Group& group) {
    auto startCoordinate = originalCoordinates[line.vertices[0]];
    auto endCoordinate = originalCoordinates[line.vertices[1]];

    auto startCell = this->calculateStructuredCell(startCoordinate);
    auto endCell = this->calculateStructuredCell(endCoordinate);

    std::vector<Cell> cells = { startCell };

    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis)
        if (startCell[axis] != endCell[axis])
            ++difference;
    
    if (difference == 2) {
        Relative startCellRelative = this->toRelative(startCell);
        Relative endCellRelative = this->toRelative(endCell);

        std::vector<std::size_t> differentAxes;
        differentAxes.reserve(2);
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (startCell[axis] != endCell[axis])
                differentAxes.push_back(axis);

        Relative candidateOne = startCellRelative;
        Relative candidateTwo = startCellRelative;
        Relative originalStartRelative = this->getRelative(startCoordinate);
        Relative originalEndRelative = this->getRelative(endCoordinate);
        candidateOne[differentAxes[0]] = endCellRelative[differentAxes[0]];
        candidateTwo[differentAxes[1]] = endCellRelative[differentAxes[1]];
        Relative startToCandidateOne = candidateOne - originalStartRelative;
        Relative startToCandidateTwo = candidateTwo - originalStartRelative;
        Relative endToCandidateOne = candidateOne - originalEndRelative;
        Relative endToCandidateTwo = candidateTwo - originalEndRelative;
        
        auto startToCandidateOneMagnitude = sqrt(pow(startToCandidateOne[0], 2) + pow(startToCandidateOne[1], 2) + pow(startToCandidateOne[2], 2));
        auto startToCandidateTwoMagnitude = sqrt(pow(startToCandidateTwo[0], 2) + pow(startToCandidateTwo[1], 2) + pow(startToCandidateTwo[2], 2));
        auto endToCandidateOneMagnitude = sqrt(pow(endToCandidateOne[0], 2) + pow(endToCandidateOne[1], 2) + pow(endToCandidateOne[2], 2));
        auto endToCandidateTwoMagnitude = sqrt(pow(endToCandidateTwo[0], 2) + pow(endToCandidateTwo[1], 2) + pow(endToCandidateTwo[2], 2));

        if (approxDir(startToCandidateOneMagnitude - startToCandidateTwoMagnitude, 0.0)
            && approxDir(endToCandidateOneMagnitude - endToCandidateTwoMagnitude, 0.0)) {
            cells.push_back(this->toCell(candidateOne));
        }
        else {
            Coordinate& startExtreme = startCoordinate;
            Coordinate& endExtreme = endCoordinate;
            Cell middleCell;
            std::size_t tries = 0;
            do {
                if (tries > 40) {
                    throw std::logic_error("Stuck in infinite loop.");
                }
                const Coordinate middlePoint = startCoordinate + (endCoordinate - startCoordinate) / 2.0;
                middleCell = this->calculateStructuredCell(middlePoint);

                if (middleCell == startCell)
                    startExtreme = middlePoint;
                else if (middleCell == endCell)
                    endExtreme = middlePoint;

                ++tries;

            } while (middleCell == startCell || middleCell == endCell);
            cells.push_back(middleCell);
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

Cell Structurer::calculateStructuredCell(const Coordinate& coordinate) const
{
    auto relativePosition = this->getRelative(coordinate);
    auto resultCell = this->getCell(coordinate);

    for (std::size_t axis = 0; axis < 3; ++axis) {
        auto distance = relativePosition[axis] - resultCell[axis];

        if (distance >= 0.5)
            ++resultCell[axis];
    }

    return resultCell;
}

}
}