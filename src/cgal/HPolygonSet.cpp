#include "HPolygonSet.h"
#include "cgal/Tools.h"

#include "utils/CoordGraph.h"
#include <boost/bimap.hpp>

namespace meshlib {
namespace cgal {

using SegmentL = CGAL::Segment_2<PK>;
using PointL = CGAL::Point_2<PK>;
using PolylineL2 = std::vector<PointL>;
using PolygonL = CGAL::Polygon_2<PK>;
using PolygonWHL = CGAL::Polygon_with_holes_2<PK>;
using PolygonSetL = CGAL::Polygon_set_2<PK>;

PointL convertPointToLocalKernel(const Point2& pE)
{
	CGAL::Cartesian_converter<K, PK> toLocal;
	return {
		toLocal(pE.x()),
		toLocal(pE.y())
	};
}

Point2 convertPointToExternalKernel(const PointL& p)
{
	CGAL::Cartesian_converter<PK, K> toExternal;
	return {
		toExternal(p.x()),
		toExternal(p.y())
	};
}

Polygon buildPolygon(const std::initializer_list<Point2> ps)
{
	Polygon res;
	for (const auto& p : ps) {
		res.push_back(p);
	}
	return res;
}

template <class T>
std::vector<CGAL::Polygon_2<T>> splitToSimplePolygons(const CGAL::Polygon_2<T>& p)
{
	assert(p.size() >= 3);
	
	boost::bimap<CGAL::Point_2<T>, CoordinateId> pToId;
	for (const auto& v : p) {
		auto it{ pToId.left.find(v) };
		if (it == pToId.left.end()) {
			pToId.insert({ v, pToId.left.size() });
		}
	}

	utils::CoordGraph cG;
	for (auto it{ p.begin() }; it != p.end(); ++it) {
		auto nIt{ std::next(it) };
		if (nIt == p.end()) {
			nIt = p.begin();
		}
		auto vIt{ pToId.left.find(*it) };
		auto vNIt{ pToId.left.find(*nIt) };
		assert(vIt != pToId.left.end());
		assert(vNIt != pToId.left.end());
		const auto cId{ vIt->second };
		const auto cNId{ vNIt->second };
		if (cId != cNId) {
			cG.addEdge(cId, cNId);
		}
	}
	
	std::vector<CGAL::Polygon_2<T>> r;
	for (const auto& cycle : cG.findCycles()) {
		if (cycle.size() < 3) {
			continue;
		}
		r.push_back({});
		for (const auto& vId : cycle) {
			auto it{ pToId.right.find(vId) };
			assert(it != pToId.right.end());
			r.back().push_back(it->second);
		}
	}

	return r;
}

template<class K>
CGAL::Polygon_2<K> removeCollinearsInPolygon(const CGAL::Polygon_2<K>& p)
{
	return tools::buildPolygonFromPolyline<K>(
		tools::buildSimplifiedPolylineFromPolygon<K>(p));
}

template <class T>
CGAL::Polygon_with_holes_2<T> removeCollinearsInPolygonWH(const CGAL::Polygon_with_holes_2<T>& p)
{
	const auto boundary{ removeCollinearsInPolygon<T>(p.outer_boundary()) };
	std::vector<CGAL::Polygon_2<T>> holes;
	for (const auto& h : p.holes()) {
		holes.push_back(removeCollinearsInPolygon<T>(h));
	}
	return { boundary, holes.begin(), holes.end() };
}

bool isSimplePWH(const PolygonWH& pwh)
{
	if (!pwh.outer_boundary().is_simple()) {
		return false;
	}
	for (const auto& hole : pwh.holes()) {
		if (!hole.is_simple()) {
			return false;
		}
	}
	return true;
}

PolygonL convertToLocalPolygon(const Polygon& p)
{
	PolygonL q;
	for (const auto& v : p) {
		q.push_back(convertPointToLocalKernel(v));
	}
	return q;
}

Polygon convertToExternalPolygon(const PolygonL& p)
{
	std::vector<Point2> pts;
	for (const auto& v : p) {
		pts.push_back(convertPointToExternalKernel(v));
	}

	std::vector<Point2> cleanedPts;
	for (auto it{ pts.begin() }; std::next(it) != pts.end(); ++it) {
		if (*it != *std::next(it)) {
			cleanedPts.push_back(*it);
		}
	}
	if (pts.back() != pts.front()) {
		cleanedPts.push_back(pts.back());
	}

	Polygon q;
	for (const auto& cP : cleanedPts) {
		q.push_back(cP);
	}
	return q;
}

PolygonWH convertToExternalPolygonWH(const PolygonWHL& p)
{	
	auto oB{ convertToExternalPolygon(p.outer_boundary()) };
	if (oB.size() < 3) {
		return PolygonWH();
	}
	std::vector<Polygon> holes;
	for (const auto& holeL : p.holes()) {
		auto hE{ convertToExternalPolygon(holeL) };
		if (hE.size() < 3) {
			continue;
		}
		holes.push_back(hE);
	}
	return {
		oB, holes.begin(), holes.end()
	};
}

HPolygonSet::HPolygonSet(const Polygon& ps)
{
	*this = HPolygonSet(convertToLocalPolygon(ps));
}

HPolygonSet::HPolygonSet(const PolygonPK& ps)
{
	if (ps.is_simple()) {
		auto sP{ removeCollinearsInPolygon(ps) };
		if (sP.size() < 3) {
			return;
		}
		CGAL::Polygon_set_2<PK>::insert(sP);
	}
	else {
		for (const auto& p : splitToSimplePolygons(ps)) {
			auto sP{ removeCollinearsInPolygon(p) };
			if (sP.size() < 3) {
				return;
			}
			CGAL::Polygon_set_2<PK>::join(sP);
		}
	}
}


HPolygonSet::HPolygonSet(const std::initializer_list<Point2>& ps)
{
	*this = HPolygonSet(buildPolygon(ps));
}

void HPolygonSet::join(const Polygon& p)
{
	CGAL::Polygon_set_2<PK>::join(convertToLocalPolygon(p));
}

void HPolygonSet::join(const HPolygonSet& pS)
{
	CGAL::Polygon_set_2<PK>::join(pS);
}

void HPolygonSet::join(const std::initializer_list<Point2>& ps)
{
	join(buildPolygon(ps));
}

void HPolygonSet::difference(const Polygon& p)
{
	CGAL::Polygon_set_2<PK>::difference(convertToLocalPolygon(p));
}

void HPolygonSet::difference(const HPolygonSet& pS)
{
	CGAL::Polygon_set_2<PK>::difference(pS);
}

void HPolygonSet::intersection(const Polygon& p)
{
	CGAL::Polygon_set_2<PK>::intersection(convertToLocalPolygon(p));
}

bool HPolygonSet::do_intersect(const Polygon& p)
{
	return CGAL::Polygon_set_2<PK>::do_intersect(convertToLocalPolygon(p));
}

bool HPolygonSet::operator==(const HPolygonSet& rhs) const
{
	auto pwh{ getPolygonsWithHoles() };
	auto rhsPWH{ rhs.getPolygonsWithHoles() };
	return pwh == rhsPWH;
}

std::size_t HPolygonSet::size() const
{
	return getPolygonsWithHoles().size();
}

PolygonWHs HPolygonSet::getPolygonsWithHoles() const
{
	std::list<PolygonWHL> local;
	CGAL::Polygon_set_2<PK>::polygons_with_holes(std::back_inserter(local));

	PolygonWHs res;
	for (const auto& pwhL : local) {
		auto pwh{ convertToExternalPolygonWH(pwhL) };
		if (pwh.outer_boundary().size() < 3) {
			continue;
		}
		if (isSimplePWH(pwh)) {
			res.push_back(pwh);		
		}
		else if (!pwh.has_holes()) {
			for (const auto& simpleP : splitToSimplePolygons(pwh.outer_boundary())) {
				res.push_back(PolygonWH{ simpleP });
			}
		}
		else {
			throw std::runtime_error(
				"Unable to convert not simple polygon with holes into simple ones."
			);
		}
	}

	return res;
}

KType HPolygonSet::area() const
{
	KType res{ 0 };
	std::list<PolygonWHL> local;
	CGAL::Polygon_set_2<PK>::polygons_with_holes(std::back_inserter(local));
	for (const auto& pwh : local) {
		res += (KType) CGAL::to_double(pwh.outer_boundary().area());
		for (const auto& hole : pwh.holes()) {
			res -= (KType) CGAL::to_double(hole.area());
		}
	}
	return res;
}

HPolygonSet::HPolygonSet(const CGAL::Polygon_set_2<PK>& ps) :
	CGAL::Polygon_set_2<PK>{ps}
{}

void HPolygonSet::clear()
{
	CGAL::Polygon_set_2<PK>::clear();
}

HPolygonSet HPolygonSet::simplifyCollinears() const
{
	CGAL::Polygon_set_2<PK> r;
	std::vector<PolygonWHL> pwhls;
	CGAL::Polygon_set_2<PK>::polygons_with_holes(std::back_inserter(pwhls));
	for (const auto& p : pwhls) {
		r.join(removeCollinearsInPolygonWH(p));
	}
	return HPolygonSet{ r };
}

bool HPolygonSet::isSimple() const
{
	const auto pwhs{ getPolygonsWithHoles() };
	return std::all_of(pwhs.begin(), pwhs.end(), isSimplePWH);
}

bool HPolygonSet::isEmpty() const
{
	return CGAL::Polygon_set_2<PK>::is_empty(); 
}

HPolygonSet::HArrangement HPolygonSet::getArrangement() const
{
	return CGAL::Polygon_set_2<PK>::arrangement();
}

}
}
