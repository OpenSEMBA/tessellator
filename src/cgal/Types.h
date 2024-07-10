#pragma once

#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>

namespace meshlib {
namespace cgal {

using KType = double;
using K = CGAL::Cartesian<KType>;

using Point1 = KType;
using Point2 = K::Point_2;
using Point3 = K::Point_3;

using Line3 = K::Line_3;
using Plane3 = K::Plane_3;

using Segment1 = std::array<Point1, 2>;
using Segments1 = std::vector<Segment1>;
using Segment2 = K::Segment_2;
using Segment3 = K::Segment_3;

using Polyline2 = std::vector<Point2>;
using Polylines2 = std::vector<Polyline2>;
using Polyline3 = std::vector<Point3>;
using Polylines3 = std::vector<Polyline3>;

using Triangle2 = CGAL::Triangle_2<K>;
using Rectangle2 = CGAL::Iso_rectangle_2<K>;
}
}
