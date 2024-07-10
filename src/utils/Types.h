#pragma once

#include "types/Mesh.h"

#include <unordered_set>

namespace meshlib {

using CoordinateMap = std::map<Coordinate, CoordinateId>;

using ElementView = const Element*;
using ElementsView = std::vector<const Element*>;

using IdSet = std::set<CoordinateId>;
using IdUSet = std::unordered_set<CoordinateId>;
using CoordinateIds = std::vector<CoordinateId>;

using Priority = int;

using Axis = std::size_t;
const Axis X{ 0 };
const Axis Y{ 1 };
const Axis Z{ 2 };

using Cell = Vector<int>;
using Relative = Coordinate;
using Relatives = std::vector<Relative>;

using CellDir = Cell::Type;
using RelativeDir = Relative::Type;

using Plane = std::pair<CellDir, Axis>;

using Linel = std::pair<Cell, Axis>;
using Surfel =  std::pair<Cell, Axis>;

using NodV = std::array<Coordinate, 1>;
using LinV = std::array<Coordinate, 2>;
using TriV = std::array<Coordinate, 3>;
using QuaV = std::array<Coordinate, 4>;
using TetV = std::array<Coordinate, 4>;
using HexV = std::array<Coordinate, 8>;

using LinVs = std::vector<LinV>;
using TriVs = std::vector<TriV>;

using NodIds = std::array<CoordinateId, 1>;
using LinIds = std::array<CoordinateId, 2>;
using TriIds = std::array<CoordinateId, 3>;
using QuaIds = std::array<CoordinateId, 4>;
using TetIds = std::array<CoordinateId, 4>;
using HexIds = std::array<CoordinateId, 8>;

using UpdateMap = std::array<std::array<std::array<int, 4>, 2>, 2>;

}

