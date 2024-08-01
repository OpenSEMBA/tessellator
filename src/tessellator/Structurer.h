#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace tessellator {

class Structurer : public utils::GridTools {
public:
    Structurer(const Grid&);

    Cell calculateStructuredCell(const Coordinate& coordinate) const;
};

}
}