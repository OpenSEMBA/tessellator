#pragma once

#include "utils/Types.h"
#include "Types.h"

#include <CGAL/Boolean_set_operations_2.h>

namespace meshlib {
namespace cgal {
namespace tools {

Point2 buildPoint2FromPoint3(const Point3&, const Axis&);
Point2 buildPoint2FromCoordinate(const Coordinate&, const Axis&);

Coordinate buildCoordinateFromPoint2(const Point2&, const double& h, const Axis& x);

template<class P>
std::vector<P> removeCollinears(const std::vector<P>& p)
{
	if (p.size() < 3) {
		return p;
	}

	std::vector<P> res;
	res.reserve(p.size());
	res.push_back(p.front());
	for (auto it = std::next(p.begin()); std::next(it) != p.end(); ++it) {
		if (!CGAL::collinear(*std::prev(it), *it, *std::next(it))) {
			res.push_back(*it);
		}
	}
	res.push_back(p.back());
	if (p.front() == p.back()) {
		auto nfIt{ std::next(p.begin()) };
		auto bIt{ p.rbegin() };
		auto bbIt{ std::next(p.rbegin()) };
		if (CGAL::collinear(*nfIt, *bIt, *bbIt)) {
			res.pop_back();
			res.front() = res.back();
		}
	}
	return res;
}

template<class K>
CGAL::Polygon_2<K> buildPolygonFromPolyline(const std::vector<typename K::Point_2>& pl)
{
	CGAL::Polygon_2<K> res;
	for (auto it = pl.begin(); std::next(it) != pl.end(); ++it) {
		res.push_back(*it);
	}
	if (pl.front() != pl.back()) {
		res.push_back(pl.back());
	}
	assert(res.size() > 2);
	return res;
}

template<class K>
std::vector<typename K::Point_2> buildSimplifiedPolylineFromPolygon(
	const CGAL::Polygon_2<K>& pb)
{
	assert(pb.size() != 0);

	std::vector<typename K::Point_2> r;
	r.reserve(pb.size() + 1);
	for (const auto& v : pb) {
		r.push_back(v);
	}
	r.push_back(r.front());
	return removeCollinears(r);
}

template<class K>
CGAL::Polygon_2<K> removeCollinearsInPolygon(const CGAL::Polygon_2<K>& p)
{
	assert(!p.is_simple());
	return buildPolygonFromPolyline<K>(
		buildSimplifiedPolylineFromPolygon<K>(p)
	);
}



}
}
}

