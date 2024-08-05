#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {

class Structurer : public utils::GridTools {
public:
    Structurer(const Mesh&);
    Mesh getMesh() const { return mesh_; };

    Cell calculateStructuredCell(const Coordinate& coordinate) const;

private:
    Mesh mesh_;

    void processLineAndAddToGroup(const Element& line, const Coordinates& originalCoordinates, Group & group);
};

}
}