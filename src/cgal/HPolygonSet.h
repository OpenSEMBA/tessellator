#pragma once

#include "Types.h"

#include <CGAL/Boolean_set_operations_2.h>

namespace meshlib {
namespace cgal {

using Polygon = CGAL::Polygon_2<K>;
using Polygons = std::vector<Polygon>;
using PolygonWH = CGAL::Polygon_with_holes_2<K>;
using PolygonWHs = std::vector<PolygonWH>;
using PolygonSet = CGAL::Polygon_set_2<K>;

using Arrangement = PolygonSet::Arrangement_2;
using Segment = PolygonSet::Arrangement_2::Traits_2::Segment_2;

using PKType = CGAL::Quotient<CGAL::MP_Float>;
using PK = CGAL::Cartesian<PKType>;
using PolygonPK = CGAL::Polygon_2<PK>;

Polygon buildPolygon(const std::initializer_list<Point2>);

class HPolygonSet : private CGAL::Polygon_set_2<PK> {
public:
	using HArrangement = CGAL::Polygon_set_2<PK>::Arrangement_2;
	using HSegment = CGAL::Polygon_set_2<PK>::Arrangement_2::Traits_2::Segment_2;

	HPolygonSet() = default;
	HPolygonSet(const Polygon&);
	HPolygonSet(const PolygonPK&);
	HPolygonSet(const std::initializer_list<Point2>&);
	
	void join(const Polygon&);
	void join(const HPolygonSet&);
	void join(const std::initializer_list<Point2>&);

	void difference(const Polygon&);
	void difference(const HPolygonSet&);
	
	void intersection(const Polygon&);
	bool do_intersect(const Polygon&);
	
	HPolygonSet simplifyCollinears() const;
	bool isSimple() const;
	bool isEmpty() const;
	KType area() const;

	bool operator==(const HPolygonSet&) const;
	std::size_t size() const;
	void clear();

	PolygonWHs getPolygonsWithHoles() const;
	HArrangement getArrangement() const;

private:
	HPolygonSet(const CGAL::Polygon_set_2<PK>&);
};

}
}
