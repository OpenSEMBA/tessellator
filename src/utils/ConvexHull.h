#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

namespace meshlib::utils {

class ConvexHull {
public:
	ConvexHull(const Coordinates* globalCoordinates);

	std::vector<CoordinateId> get(const IdSet& vertexIds) const;

private:
	const Coordinates* globalCoords_ = nullptr;
};

}
