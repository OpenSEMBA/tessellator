#include "Structurer.h"

namespace meshlib {
namespace tessellator {
using namespace utils;

Structurer::Structurer(const Grid& grid) : GridTools(grid)
{

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