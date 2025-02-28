#pragma once

#include "utils/Types.h"

#include <boost/array.hpp>

namespace meshlib {

using SliceNumber = CellDir;
using GridPlane = std::pair<Axis, SliceNumber>;

using ArrayIndex = boost::array<CellDir, 2>;

struct CellIndex {
	Cell ijk;
	Axis axis;

	ArrayIndex getArrayIndex() const {
		return {
			ijk[(axis + 1) % 3],
			ijk[(axis + 2) % 3]
		};
	}
	SliceNumber getSliceNumber() const {
		return ijk[axis];
	}
};

}