#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

namespace meshlib {
namespace cgal {

class ConvexHull {
public:
	ConvexHull(const Coordinates* globalCoordinates);

	std::vector<CoordinateId> get(const IdSet& vertexIds) const;

private:
	const Coordinates* globalCoords_ = nullptr;
	static const double COPLANARITY_ANGLE_TOLERANCE;
	
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef K::Point_2 Point_2;
	typedef CGAL::Convex_hull_traits_adapter_2<K,
		CGAL::Pointer_property_map<Point_2>::type > Convex_hull_traits_2;

	typedef std::map<Point_2, CoordinateId> IndexPointToId;

	IndexPointToId buildPointsInIndex(const IdSet& inIds) const;
};

}
}
