#pragma once
#include <map>
#include "utils/Types.h"
#include "utils/CoordGraph.h"

namespace meshlib {
using namespace utils;

float const FLOAT_EPSILON = float(1e-6);

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

typedef std::pair<CoordinateId, CoordinateId>   Segment;

}