#include "Slice.h"

#include <CGAL/Boolean_set_operations_2.h>

#include "cgal/Tools.h"

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>


namespace meshlib {
namespace tessellator {
namespace filler {

using Bbox2 = CGAL::Bbox_2;

using cgal::tools::buildCoordinateFromPoint2;
using cgal::tools::removeCollinears;

bool isAlignedWithAxis(const Segment2& seg)
{
	for (auto d{ 0 }; d < 2; ++d) {
		const KType& p0d{ seg.vertex(0).cartesian(d) };
		const KType& p1d{ seg.vertex(1).cartesian(d) };
		if (p0d != p1d) {
			continue;
		}
		double x0{ CGAL::to_double(p0d) };
		double x1{ CGAL::to_double(p1d) };
		if (std::floor(x0) == x0 && std::floor(x0) == x1) {
			return true;
		}
	}
	return false;
}

Polylines2 removeSegmentsContainedInAnyAxis(const Polyline2& p)
{
	if (p.size() == 0) {
		return {};
	}
	if (p.size() == 1) {
		return { p };
	}

	Polylines2 res;
	bool newPolyline{ true };
	for (auto it{ p.begin() }; std::next(it) != p.end(); ++it) {
		if (isAlignedWithAxis({ *it, *std::next(it) })) {
			newPolyline = true;
			continue;
		}
		if (newPolyline) {
			res.push_back({ *it });
		}
		res.back().push_back(*std::next(it));
		newPolyline = false;
	}
	return res;
}

std::array<ArrayIndex, 2> buildMinAndMaxArrayIndices(const Bbox2& bbox)
{
	ArrayIndex minAI{
		std::numeric_limits<CellDir>::max(),
		std::numeric_limits<CellDir>::max()
	};
	ArrayIndex maxAI{
		std::numeric_limits<CellDir>::min(),
		std::numeric_limits<CellDir>::min()
	};
	for (auto d{ 0 }; d < 2; ++d) {
		if ((CellDir)std::floor(bbox.min(d)) < minAI[d]) {
			minAI[d] = (CellDir)std::floor(bbox.min(d));
		}
		if ((CellDir)std::floor(bbox.max(d)) > maxAI[d]) {
			maxAI[d] = (CellDir)std::ceil(bbox.max(d));
		}
	}
	return { minAI, maxAI };
}

bool isCellCrossedByPolylines(const Polylines2& pls, const ArrayIndex& idx) 
{
	auto cF{ buildCellFace(idx) };
	for (const auto& pl : pls) {
		for (auto it{ pl.begin() }; std::next(it) != pl.end(); ++it) {
			Segment2 seg{ *it, *std::next(it) };
			if (CGAL::do_intersect(cF, seg)) {
				auto intersection{*CGAL::intersection(cF, seg)};
				if (!boost::get<Point2 >(&intersection)) {
					return true;
				}
			}
		}
	}
	return false;
}

Bbox2 getBoundingBox(const Polylines2& pls) 
{
	assert(!pls.empty());

	std::array<KType, 2> min{
		std::numeric_limits<KType>::max(),
		std::numeric_limits<KType>::max()
	};
	std::array<KType, 2> max{
		std::numeric_limits<KType>::min(),
		std::numeric_limits<KType>::min()
	};
	for (const auto& pl : pls) {
		assert(!pl.empty());
		for (const auto& p : pl) {
			for (int d{ 0 }; d < 2; d++) {
				if (p.cartesian(d) < min[d]) {
					min[d] = p.cartesian(d);
				}
				if (p.cartesian(d) > max[d]) {
					max[d] = p.cartesian(d);
				}
			}
		}
	}
	return Bbox2{ 
		CGAL::to_double(min[X]), 
		CGAL::to_double(min[Y]), 
		CGAL::to_double(max[X]), 
		CGAL::to_double(max[Y]) 
	};
}

std::vector<ArrayIndex> faceIntersections(const Polylines2& pls)
{
	std::vector<ArrayIndex> res;
	if (pls.empty()) {
		return res;
	}

	auto minMax{ buildMinAndMaxArrayIndices(getBoundingBox(pls)) };
	res.reserve(minMax.size());
	for (int i{ minMax[0][0]}; i < minMax[1][0]; ++i) {
		for (int j{ minMax[0][1]}; j < minMax[1][1]; ++j) {
			const ArrayIndex idx{ i, j };
			if (isCellCrossedByPolylines(pls, idx)) {
				res.push_back(idx);
			}
		}
	}
	return res;
}

std::vector<ArrayIndex> faceIntersections(const Triangle2& t)
{
	std::vector<ArrayIndex> res;
	if (t.is_degenerate()) {
		std::cerr << "Triangle is degenerated:" << t << std::endl;
		return res;
	}
	auto minMax{ buildMinAndMaxArrayIndices(t.bbox()) };
	res.reserve(minMax.size());
	for (int i{ minMax[0][0] }; i < minMax[1][0]; ++i) {
		for (int j{ minMax[0][1]}; j < minMax[1][1]; ++j) {
			const ArrayIndex idx{ i, j };
			if (isCellCrossedByTriangle(t, idx)) {
				res.push_back(idx);
			}
		}
	}
	return res;
}

void updateContourIndex(Slice::ContourIndexSet& cIS, const Polyline2& p)
{
	auto nonAligned{ faceIntersections(removeSegmentsContainedInAnyAxis(p)) };
	cIS.insert(nonAligned.begin(), nonAligned.end());
}

void Slice::add(const Polylines2& polylines, const Priority& pr)
{
	SliceData& sd = data_[pr];
	
	for (const auto& p : polylines) {
		if (p.size() == 1) {
			continue;
		}
		auto r{ removeSegmentsContainedInAnyAxis(p) };
		sd.lines.insert(sd.lines.end(), r.begin(), r.end());
	}
}

FillingState::FillingState(const FillingType& t) :
	type{ t },
	priority_{ 0 }
{
	assert(t != FillingType::Full);
}

FillingState::FillingState(const Priority& p) :
	type{FillingType::Full},
	priority_{p}
{}

Priority FillingState::getPriority() const
{
	assert(type == FillingType::Full);
	return priority_;
}

void Slice::removeInSuperiorPriorities(const Priority& pr)
{
	for (auto& [p, s] : data_) {
		if (pr < p) {
			data_[pr].surfaces.difference(s.surfaces);
			data_[pr].surfaces = data_[pr].surfaces;
		}
	}
}

void Slice::addAsPolygon(const Polylines2& polylines, const Priority& pr)
{
	SliceData& sd = data_[pr];

	std::list<Polygon> outerBounds, holes;
	for (const auto& p : polylines) {
		if (p.size() <= 3) {
			continue;
		}
		auto polygon{ tools::buildPolygonFromPolyline<K>(p) };
		if (!polygon.is_simple()) {
			continue;
		}
		if (polygon.is_counterclockwise_oriented()) {
			outerBounds.push_back(polygon);
		}
		else {
			polygon.reverse_orientation();
			holes.push_back(polygon);
		}
	}

	for (const auto& polygon : outerBounds) {
		sd.surfaces.join(polygon);
	}
	for (const auto& polygon : holes) {
		sd.surfaces.difference(polygon);
	}
	removeInSuperiorPriorities(pr);
}

Polyline2 buildPolylineFromPolygon(const Polygon& p)
{
	Polyline2 r(p.begin(), p.end());
	r.push_back(*p.begin());
	return r;
}

void Slice::add(const HPolygonSet& polygons, const Priority& pr)
{
	if (polygons.isEmpty()) {
		return;
	}

	SliceData& sd = data_[pr];
	sd.surfaces.join(polygons);

	removeInSuperiorPriorities(pr);
}

void Slice::mergeLines(const Slice& lhs)
{
	for (const auto& [pr, sd] : lhs.data_) {
		assert(sd.surfaces.isEmpty());
		auto& rhsLines = this->data_[pr].lines;
		rhsLines.insert(rhsLines.end(), sd.lines.begin(), sd.lines.end());
	}
	auto& contourFlag = this->nonEdgeAlignedContourIndices_;
	contourFlag = lhs.nonEdgeAlignedContourIndices_;
}

TriV buildTriVFromFace(const CDT::Face f, Axis x, Height h) {
	TriV tri;
	for (int i = 0; i < 3; i++) {
		tri[i] = buildCoordinateFromPoint2(
			f.vertex(i)->point(), h, x);
	}
	return tri;
}

Triangle2 buildTriangle2FromFace(const CDT::Face& f)
{
	std::vector<Point2> v(3);
	for (int i{ 0 }; i < 3; ++i) {
		v[i] = f.vertex(i)->point();
	}
	return { v[0], v[1], v[2] };
}

TriVs buildTriVsFromTriangulation(const CDT& cdt, Axis x, Height h)
{
	TriVs res;
	res.reserve(cdt.number_of_faces());

	for (const auto& f : cdt.finite_face_handles()) {
		if (f->info().in_domain()) {
			res.push_back(buildTriVFromFace(*f, x, h));
		}
	}
	return res;
}

LinVs buildLinVsFromPolyline(
	const std::vector<Polyline2>& pls,
	Axis axis,
	double height)
{
	LinVs res;
	for (const auto& pl : pls) {
		for (auto vIt{ pl.begin() }; std::next(vIt) != pl.end(); ++vIt) {
			LinV lin{
				buildCoordinateFromPoint2(*vIt,            height, axis),
				buildCoordinateFromPoint2(*std::next(vIt), height, axis),
			};
			res.push_back(lin);
		}
	}
	return res;
}

Polygon buildPolygonFromFaceTriIntersection(
	const Rectangle2& i2, const Triangle2& t2)
{
	auto intResult{ CGAL::intersection(i2, t2) };
	if (!intResult) {
		return Polygon();
	}
	Polygon r;
	if (const auto& t = boost::get<Triangle2>(&*intResult)) {
		for (int i{ 0 }; i < 3; ++i) {
			r.push_back(t->vertex(i));
		}
	}
	else if (const auto& ps = boost::get<std::vector<Point2>>(&*intResult)) {
		assert(ps->size() > 3);
		for (std::size_t i{ 0 }; i < ps->size(); ++i) {
			r.push_back((*ps)[i]);
		}
	}
	return r;
}

Polyline2 buildCellLineIntersection(
	const Rectangle2& c,
	const Polyline2& pl)
{
	Polyline2 r;
	for (auto vIt{ pl.begin() }; std::next(vIt) != pl.end(); ++vIt) {
		Segment2 s{ *vIt, *std::next(vIt) };
		auto intResult{ CGAL::intersection(c, s) };
		if (!intResult) {
			continue;
		};
		if (const auto& p = boost::get<Segment2>(&*intResult)) {
			if ((*p)[0] == (*p)[1]) {
				continue;
			}
			if (r.empty()) {
				r.push_back((*p)[0]);
			}
			r.push_back((*p)[1]);
		}
	}
	return r;
}

void Slice::SliceData::buildLineMaps()
{
	for (const auto& l : lines) {
		LineMap lMap;
		for (auto it{ l.begin() }; std::next(it) != l.end(); ++it) {
			Polyline2 segment{ *it, *std::next(it) };
			for (const auto& idx : faceIntersections({ segment })) {
				lMap[idx].push_back(it);
			}
		}
		lineMaps.push_back(lMap);
	}
}

void Slice::SliceData::buildSurfaceMaps()
{
	for (const auto& triangulation : triangulations) {
		for (const auto& f : triangulation.finite_face_handles()) {
			if (!f->info().in_domain()) {
				continue;
			}
			for (const auto& idx : faceIntersections(buildTriangle2FromFace(*f))) {
				assert(isValidFace(*f));
				trianglesMaps[idx].push_back(&*f);
			}
		}
	}
}

bool Slice::SliceData::isEmpty() const
{
	return
		lines.size() == 0 &&
		surfaces.size() == 0 &&
		triangulations.size() == 0;
}

void Slice::SliceData::buildMaps()
{
	buildSurfaceMaps();
	buildLineMaps();
}

void Slice::buildTriangulations()
{
	for (auto& [pr, sd] : data_) {
		sd.triangulations = buildCDTsFromPolygonSet(sd.surfaces);
	}
}

void Slice::simplifySurfaces()
{
	auto it = data_.begin();
	for (; it != data_.end(); ) {
		if (it->second.isEmpty()) {
			it = data_.erase(it);
		}
		else {
			++it;
		}
	}
	
	for (auto& [pr, sd] : data_) {
		sd.surfaces = sd.surfaces.simplifyCollinears();
	}
}

void Slice::cleanSurfaces()
{
	for (auto& [pr, sd] : data_) {
		sd.surfaces.clear();
	}
}

void Slice::buildSearchMap() 
{
	for (auto& it : data_) {
		it.second.buildMaps();
	}

	for (const auto& it : data_) {
		for (const auto& pwh : it.second.surfaces.getPolygonsWithHoles()) {
			updateContourIndex(nonEdgeAlignedContourIndices_,
				buildPolylineFromPolygon(pwh.outer_boundary()));
			for (const auto& h : pwh.holes()) {
				updateContourIndex(nonEdgeAlignedContourIndices_,
					buildPolylineFromPolygon(h));
			}
		}
		for (const auto& pl : it.second.lines) {
			updateContourIndex(nonEdgeAlignedContourIndices_, pl);
		}
	}
}

void Slice::fillSurfaces(
	FaceFilling& r,
	const ArrayIndex& idx) const
{
	//assert(getFillingState(idx).partial());
	for (auto& [pr, sd] : data_) {
		const auto& surface{ sd.triangulations };
		const auto& searchMap{ sd.trianglesMaps };
		auto it{ searchMap.find(idx) };
		if (it == searchMap.end()) {
			// This may happen when a cell is crossed by 
			// a polyline but does not contain surfaces.
			continue;
		}
		auto trisInFace{ buildPolygonSetFromCDT(it->second) };
		auto cellFacePolygon{ buildCellFacePolygon(idx) };
		assert(trisInFace.do_intersect(cellFacePolygon));
		trisInFace.intersection(cellFacePolygon);
		r.tris.emplace(pr, trisInFace);
	}
}

void Slice::fillLines(FaceFilling& r, const ArrayIndex& idx) const 
{
	auto cellFace{ buildCellFace(idx) };
	for (const auto& [pr, sd] : data_) {
		for (const auto& lineMap : sd.lineMaps) {
			auto it{ lineMap.find(idx) };
			if (it == lineMap.end()) {
				continue;
			}
			const auto& plRange{ it->second };
			for (const auto& it : plRange) {
				Polyline2 polylineChunk{ *it, *std::next(it) };
				auto p{ buildCellLineIntersection(cellFace, polylineChunk) };
				if (!p.empty()) {
					r.lins[pr].push_back(p);
				}
			}
		}
	}
}

FillingState Slice::getFillingState(const ArrayIndex& idx) const
{
	if (nonEdgeAlignedContourIndices_.count(idx) != 0) {
		return FillingState{ FillingType::Partial };
	}	
	for (const auto& [pr, sd] : data_) {
		if (sd.trianglesMaps.count(idx) != 0) {
			return FillingState{ pr };
		}
	}
	return FillingState{ FillingType::Empty };
}

FaceFilling Slice::getFaceFilling(const ArrayIndex& idx) const
{
	FaceFilling res;
	fillSurfaces(res, idx);
	fillLines(res, idx);
	return res;
}

TriVs Slice::buildAllTriVs(const Priority& pr, Axis x, Height h) const
{
	TriVs res;
	auto it = data_.find(pr);
	if (it == data_.end()) {
		return res;
	}
	
	for (const auto& triangulation : it->second.triangulations) {
		for (const auto& f : triangulation.finite_face_handles()) {
			if (f->info().in_domain()) {
				res.push_back(buildTriVFromFace(*f, x, h));
			}
		}
	}
	return res;
}

LinVs Slice::buildAllLinVs(const Priority& pr, Axis x, Height h) const
{
	LinVs res;
	auto it = data_.find(pr);
	if (it == data_.end()) {
		return res;
	}

	for (const auto& pl : it->second.lines) {
		for (auto vIt{ pl.begin() }; std::next(vIt) != pl.end(); ++vIt) {
			LinV lin{
				buildCoordinateFromPoint2(*vIt, h, x),
				buildCoordinateFromPoint2(*std::next(vIt), h, x)
			};
			res.push_back(lin);
		}
	}
	return res;
}

}
}
}
