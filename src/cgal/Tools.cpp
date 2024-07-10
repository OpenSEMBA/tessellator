#include "Tools.h"

namespace meshlib {
namespace cgal {
namespace tools {


Point2 buildPoint2FromCoordinate(const Coordinate& c, const Axis& axis)
{
	return {
		c[(axis + 1) % 3],
		c[(axis + 2) % 3]
	};
}

Point2 buildPoint2FromPoint3(const Point3& v, const Axis& x)
{
	return {
		v[(x + 1) % 3],
		v[(x + 2) % 3]
	};
}

Coordinate buildCoordinateFromPoint2(const Point2& v, const double& h, const Axis& x)
{
	Coordinate r;
	r[x] = h;
	r[(x + 1) % 3] = v.x();
	r[(x + 2) % 3] = v.y();
	return r;
}


}
}
}
