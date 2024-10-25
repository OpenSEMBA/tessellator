#include "FillerTools.h"

#include "utils/CoordGraph.h"
#include <boost/bimap.hpp>

namespace meshlib {
namespace tessellator {
namespace filler {

using namespace cgal;


Rectangle2 buildCellFace(const ArrayIndex& c)
{
	const double x(c[0]);
	const double y(c[1]);
	return {
		Point2(x,       y),
		Point2(x + 1.0, y + 1.0)
	};
}

bool isCellCrossedByTriangle(const Triangle2& t, const ArrayIndex& idx)
{
	auto cF{ buildCellFace(idx) };
	if (t.is_degenerate()) {
		return false;
	}
	if (CGAL::do_intersect(cF, t)) {
		auto iP{ *CGAL::intersection(cF, t) };
		if (!boost::get<Point2 >(&iP)) {
			return true;
		}
	}
	return false;
}

Polygon buildCellFacePolygon(const ArrayIndex& idx)
{
	const double x(idx[0]);
	const double y(idx[1]);
	Polygon res;
	res.push_back({ x,       y });
	res.push_back({ x + 1.0, y });
	res.push_back({ x + 1.0, y + 1.0 });
	res.push_back({ x,       y + 1.0 });
	return res;
}

void mark_domains(
	CDT& ct,
	CDT::Face_handle start,
	int index,
	std::list<CDT::Edge>& border)
{
	if (start->info().nesting_level != -1) {
		return;
	}
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);
	while (!queue.empty()) {
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if (fh->info().nesting_level == -1) {
			fh->info().nesting_level = index;
			for (int i = 0; i < 3; i++) {
				CDT::Edge e(fh, i);
				CDT::Face_handle n = fh->neighbor(i);
				if (n->info().nesting_level == -1) {
					if (ct.is_constrained(e)) border.push_back(e);
					else queue.push_back(n);
				}
			}
		}
	}
}

void mark_domains(CDT& cdt)
{
	for (CDT::Face_handle f : cdt.all_face_handles()) {
		f->info().nesting_level = -1;
	}
	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);
	while (!border.empty()) {
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);
		if (n->info().nesting_level == -1) {
			mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
		}
	}
}

CDT buildCDTFromPolygonWH(const cgal::PolygonWH& pWH)
{
	CDT nT;
	
	nT.insert_constraint(pWH.outer_boundary().begin(), pWH.outer_boundary().end(), true);
	for (const auto& hole : pWH.holes()) {
		nT.insert_constraint(hole.begin(), hole.end(), true);
	}
	mark_domains(nT);
	return nT;
}

CDTs buildCDTsFromPolygonSet(const cgal::HPolygonSet& surfaces) 
{
	CDTs r;
	for (const auto& pWH : surfaces.getPolygonsWithHoles()) {
		r.push_back(buildCDTFromPolygonWH(pWH));
	}
	return r;
}

HPolygonSet buildPolygonSetFromContour(const Polygons& contours)
{
	std::vector<const Polygon*> polygonsToJoin, polygonsToSubstract;
	for (const auto& cp : contours) {
		if (cp.is_counterclockwise_oriented()) {
			polygonsToJoin.push_back(&cp);
		}
		else {
			polygonsToSubstract.push_back(&cp);
		}
	}

	HPolygonSet res;
	for (const auto& cp : polygonsToJoin) {
		res.join(*cp);
	}
	for (const auto& cp : polygonsToSubstract) {
		auto p{ *cp };
		p.reverse_orientation();
		res.difference(p);
	}
	return res;
}

bool isValidFace(const CDT::Face& f)
{
	for (auto i{ 0 }; i < 3; ++i) {
		if (f.vertex(i)->point() == f.vertex((i + 1) % 3)->point()) {
			return false;
		}
	}
	return true;
}

Polygons buildContourFromGraph(
	const utils::CoordGraph& cG,
	const boost::bimap<Point2, CoordinateId>& pToId)
{
	Polygons ps;
	for (const auto& cycle : cG.getBoundaryGraph().findCycles()) {
		ps.push_back({});
		for (const auto& vId : cycle) {
			ps.back().push_back(pToId.right.find(vId)->second);
		}
	}
	return ps;
}

Polygons buildContourMap2(const std::vector<const CDT::Face*>& tris)
{
	boost::bimap<Point2, CoordinateId> pToId;
	for (const auto& f : tris) {
		for (auto i{ 0 }; i < 3; ++i) {
			const auto& v{ f->vertex(i)->point() };
			auto it{ pToId.left.find(v) };
			if (it == pToId.left.end()) {
				pToId.insert({ v, pToId.left.size() });
			}
		}
	}
	
	utils::CoordGraph cG;
	for (const auto& f : tris) {
		for (auto i{ 0 }; i < 3; ++i) {
			auto vIt{ pToId.left.find(f->vertex(i)->point()) };
			auto vNIt{ pToId.left.find(f->vertex((i+1)%3)->point()) };
			assert(vIt != pToId.left.end());
			assert(vNIt != pToId.left.end());
			cG.addEdge(vIt->second, vNIt->second);
		}
	}
		
	return buildContourFromGraph(cG, pToId);
}

HPolygonSet buildPolygonSetFromCDT(
	const std::vector<const CDT::Face*>& tris)
{
	return buildPolygonSetFromContour(
		buildContourMap2(tris));
}

Polygons buildContourMap3(const Polyhedron& p, const Axis& x)
{
	boost::bimap<Point2, CoordinateId> pToId;
	for (const auto& f : p.facet_handles()) {
		auto v = f->facet_begin();
		do {
			auto point{
				tools::buildPoint2FromPoint3(v->vertex()->point(), x)
			};
			auto it{ pToId.left.find(point) };
			if (it == pToId.left.end()) {
				pToId.insert({point, pToId.left.size() });
			}
		} while (++v != f->facet_begin());
	}

	utils::CoordGraph cG;
	for (const auto& f : p.facet_handles()) {
		auto v = f->facet_begin();
		do {
			auto pIt{ pToId.left.find(
				tools::buildPoint2FromPoint3(
					v->vertex()->point(), x))};
			auto vN{ std::next(v) };
			auto pNIt{ pToId.left.find(
				tools::buildPoint2FromPoint3(
					vN->vertex()->point(), x))};
			assert(pIt != pToId.left.end());
			assert(pNIt != pToId.left.end());
			CoordinateId cId{ pIt->second };
			CoordinateId cIdN{ pNIt->second };
			cG.addEdge(cId, cIdN);
		} while (++v != f->facet_begin());
	}

	return buildContourFromGraph(cG, pToId);
}

HPolygonSet buildPolygonSetFromPolyhedron(
	const Polyhedron& p, const Axis& x)
{
	return buildPolygonSetFromContour(
		buildContourMap3(p, x)
	);
}


}
}
}