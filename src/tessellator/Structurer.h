#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {

class Structurer : public utils::GridTools {
public:
    Structurer(const Mesh&);
    Mesh getMesh() const { return mesh_; };

    Cell calculateStructuredCell(const Coordinate& relativeCoordinate) const;

private:
    Mesh mesh_;

    void processLineAndAddToGroup(const Element& line, const Coordinates& originalRelativeCoordinates, Group & group);
    std::size_t calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell);
    Cell calculateMiddleCellBetweenTwoCoordinates(Coordinate& startExtreme, Coordinate& endExtreme, Relative& step);

};


}
}