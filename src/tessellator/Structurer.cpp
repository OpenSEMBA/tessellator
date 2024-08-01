#include "Structurer.h"

namespace meshlib {
namespace tessellator {
using namespace utils;

Structurer::Structurer(const Mesh& inputMesh) : GridTools(inputMesh.grid)
{
    mesh_.grid = inputMesh.grid;

    mesh_.coordinates.resize(inputMesh.coordinates.size());
    for (std::size_t i = 0; i < mesh_.coordinates.size(); ++i)
    {
        auto structuredCell = this->calculateStructuredCell(inputMesh.coordinates[i]);
        auto relativePosition = this->toRelative(structuredCell);
        mesh_.coordinates[i] = relativePosition;
    }

    mesh_.groups.resize(inputMesh.groups.size());

    for (std::size_t i = 0; i < mesh_.groups.size(); ++i) {
        auto& inputGroup = inputMesh.groups[i];
        auto& meshGroup = mesh_.groups[i];
        meshGroup.elements.resize(inputGroup.elements.size());
        for (std::size_t j = 0; j < inputGroup.elements.size(); ++j) {
            meshGroup.elements[j] = inputGroup.elements[j];
        }
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