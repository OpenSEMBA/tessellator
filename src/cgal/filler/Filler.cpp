#include "Filler.h"

#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif

#include "cgal/PolyhedronTools.h"
#include "cgal/Tools.h"
#include "cgal/Manifolder.h"

#include "utils/MeshTools.h"

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_slicer.h>

#include <CGAL/Polygon_mesh_processing/orientation.h>

namespace meshlib {
namespace tessellator {
namespace filler {

using namespace cgal;
using namespace polyhedronTools;
using namespace tools;
namespace PMP = CGAL::Polygon_mesh_processing;

using Primitive = CGAL::AABB_face_graph_triangle_primitive<Polyhedron>;
using Traits = CGAL::AABB_traits<K, Primitive>;
using LineIntersectionsTree = CGAL::AABB_tree<Traits> ;
using LineE3_intersection = boost::optional< LineIntersectionsTree::Intersection_and_primitive_id<Line3>::Type > ;

using HGSP = CGAL::AABB_halfedge_graph_segment_primitive<Polyhedron>;
using AABB_traits = CGAL::AABB_traits<K, HGSP>;
using PMSlicerTree = CGAL::AABB_tree<AABB_traits>;
using PMSlicer = CGAL::Polygon_mesh_slicer<Polyhedron, K>;

enum class SlicingMode {
	Surface,
	Volume
};

void log(const std::string& msg, std::size_t level = 0)
{
	std::cout << "[Filler] ";
	for (std::size_t i = 0; i < level; i++) {
		std::cout << "-- ";
	}

	std::cout << msg << std::endl;
}

struct FillerPolyhedrons {
	Polyhedron volumes;
	Polyhedron surfaces;
	Polyhedron aligned;
};

Plane3 buildSlicingPlane(const Axis& axis, const Height& offset) 
{
	switch (axis)
	{
	case X:
		return Plane3(1.0, 0.0, 0.0, -offset);
	case Y:
		return Plane3(0.0, 1.0, 0.0, -offset);
	case Z:
		return Plane3(0.0, 0.0, 1.0, -offset);
	default:
		throw std::runtime_error("Invalid axis building plane");
	}
}

Polyline2 convertPolyline3ToPolyline2(const Polyline3& pl, const Axis& x)
{
	Polyline2 pg;
	pg.reserve(pl.size());
	for (const auto& v : pl) {
		pg.push_back(buildPoint2FromPoint3(v, x));
	}
	if (pg.size() <= 1) {
		return pg;
	}

	bool closedPolyline{ pg.front() == pg.back() };
	if (closedPolyline) {
		pg.pop_back();
	}
	std::rotate(
		pg.begin(),
		std::min_element(pg.begin(), pg.end()),
		pg.end()
	);
	if (closedPolyline) {
		pg.push_back(pg.front());
	}
	return pg;
}

std::array<std::map<SliceNumber, Polylines2>, 3> 
buildGridPlanesPolylines(const Polyhedron& m, const Grid& g)
{
	std::array<std::map<SliceNumber, Polylines2>, 3> res;
	
	if (m.empty()) {
		return res;
	}

	PMSlicerTree tree{ edges(m).first, edges(m).second, m };
	PMSlicer slicer(m, tree);

	const std::array<Axis, 3> axis{ X, Y, Z };

	std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
		std::execution::par,
#endif
		axis.begin(), axis.end(),
		[&](const auto& x) {
			for (std::size_t i{ 0 }; i < g[x].size(); ++i) {
				Polylines3 pl3s;
				slicer(buildSlicingPlane(x, (Height)i), std::back_inserter(pl3s));
				for (const auto& pl3 : pl3s) {
					auto pl{ removeCollinears(convertPolyline3ToPolyline2(pl3, x)) };
					if (pl.size() < 2) {
						continue;
					}
					res[x][(int)i].push_back(pl);
				}
			}
		}
	);
	
	return res;
}

void sliceNonAlignedByGrid(
	Filler::GridSlices& slices,
	const Polyhedron& m, 
	const Grid& g, 
	const Priority& priority,
	const SlicingMode mode)
{
	const auto polyLines{ buildGridPlanesPolylines(m, g) };
	const std::array<Axis, 3> axis{ X, Y, Z };

	std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
		std::execution::par,
#endif
		axis.begin(), axis.end(),
		[&](const auto& x) {
			for (const auto& [i, lines] : polyLines[x]) {
				if (mode == SlicingMode::Surface) {
					slices[x][i].add(lines, priority);
				}
				else {
					slices[x][i].addAsPolygon(lines, priority);
				}
			}
		}
	);	
}

Polygon buildPolygonFromFace(const Polyhedron::Facet& f, const Axis& x)
{
	assert(f.size() > 2);
	Polygon res;
	auto v = f.facet_begin();
	do {
		res.push_back(buildPoint2FromPoint3(v->vertex()->point(), x));
	} while (++v != f.facet_begin());
	return res;
}

std::array<std::map<SliceNumber, HPolygonSet>, 3>
buildGridPlanesPolygons(const Polyhedron& m, const Grid& g)
{
	std::array<std::map<SliceNumber, HPolygonSet>, 3> res;
	if (m.empty()) {
		return res;
	}

	Polyhedron mM;
	CGAL::copy_face_graph(m, mM);
	
	using FacePropertyTag = CGAL::dynamic_face_property_t<GridPlane>;
	using FaceFilteredPolyhedron = CGAL::Face_filtered_graph<Polyhedron>;
	
	auto gridPlaneMap{ get(FacePropertyTag(), mM) };
	std::set<GridPlane> usedGridPlanes;
	for (auto f{ mM.facets_begin() }; f != mM.facets_end(); ++f) {
		auto cP{ getFaceCartesianPlane(*f) };
		usedGridPlanes.insert(cP);
		put(gridPlaneMap, f, cP);
	}

	for (const auto& usedGridPlane : usedGridPlanes) {
		FaceFilteredPolyhedron filtered(mM, usedGridPlane, gridPlaneMap);
		Polyhedron pPlane;
		CGAL::copy_face_graph(filtered, pPlane);
		const auto& x{ usedGridPlane.first };
		const auto& sliceNumber{ usedGridPlane.second };
		
		auto pS{ buildPolygonSetFromPolyhedron(pPlane, x) };
		if (pS.size() != 0) {
			res[x][sliceNumber].join(pS);
		}
	}
	return res;
}

bool isFacetCCWOriented(const Polyhedron::Facet& f)
{
	const Axis x{ getFaceCartesianPlane(f).first };
	Polygon p;
	auto v{ f.facet_begin() };
	do { 
		p.push_back(buildPoint2FromPoint3(v->vertex()->point(), x));
	} while (++v != f.facet_begin());
	return p.is_clockwise_oriented();
}

Polyhedron makeFacesCCWOriented(const Polyhedron& sm)
{
	Polyhedron r{ sm };
	Polyhedron sCCW;
	reassignFacetsWithPredicate(sCCW, r, isFacetCCWOriented);
	PMP::reverse_face_orientations(sCCW);
	CGAL::copy_face_graph(sCCW, r);
	return r;
}

void sliceAlignedByGrid(
	Filler::GridSlices& slices,
	const Polyhedron& m,
	const Grid& g,
	const Priority& priority)
{
	
	auto polygons{ buildGridPlanesPolygons(makeFacesCCWOriented(m), g)};
	const std::array<Axis, 3> axis{ X, Y, Z };

	std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
		std::execution::par,
#endif
		axis.begin(), axis.end(),
		[&](const auto& x) {
			for (const auto& [i, polygon] : polygons[x]) {
				slices[x][i].add(polygon, priority);
			}
		}
	);
}

Priority Filler::getGroupPriority(const GroupId& gId) const
{
	if (gId < groupPriorities_.size()) {
		return groupPriorities_[gId];
	}
	if (groupPriorities_.size() == 0) {
		return Priority(gId);
	}
	throw std::runtime_error("Unable to get group priority.");
}

void mergeGridSliceLines(Filler::GridSlices& rhs, const Filler::GridSlices& lhs)
{
	for (const auto& x : {X, Y, Z}) {
		for (const auto& [num, slice]: lhs[x]) {
			rhs[x][num].mergeLines(slice);
			
		}
	}
}

Line3 buildLineQuery(const ArrayIndex& ij, const Axis& x)
{
	VecD ini, end;
	ini(x) = 0.0;
	ini[(x + 1) % 3] = (double) ij[0];
	ini[(x + 2) % 3] = (double) ij[1];
	end = ini;
	end(x) = (double) 1.0;
	
	return {
		Point3{ini[X], ini[Y], ini[Z]},
		Point3{end[X], end[Y], end[Z]},
	};
}

Segments1 convertToSegments1(const std::list<LineE3_intersection>& inters, const Axis& x)
{
	if (inters.empty()) {
		return Segments1();
	}

	std::set<Segment1> segs;
	for (auto it = inters.begin(); it != inters.end(); ++it) {		
		if (const auto& e = boost::get<Segment3>(&((*it)->first))) {
			segs.insert(
				Segment1{ 
					Point1{(*e)[0][(int)x]},
					Point1{(*e)[1][(int)x]}
				}
			);
		}
	}

	Segments1 res;
	res.reserve(segs.size());
	for (const auto& seg : segs) {
		if (res.empty()) {
			res.push_back(seg);
		}
		if (res.back()[1] == seg[0]) {
			res.back()[1] = seg[1];
		}
	}
	
	return res;
}

void buildSegmentsArray(
	Filler::GridSegmentsArray& arr,
	const Polyhedron& p,
	const Grid& g,
	const Priority& pr)
{
	LineIntersectionsTree tree{ faces(p).first, faces(p).second, p };

	for (const auto& x : { X, Y, Z }) {
		const auto y{ (x + 1) % 3 };
		const auto z{ (x + 2) % 3 };
		for (auto i{ 0 }; i < g[y].size(); ++i) {
			for (auto j{ 0 }; j < g[z].size(); ++j) {
				const ArrayIndex ij{ i,j };
				std::list<LineE3_intersection> intersections;
				tree.all_intersections(
					buildLineQuery(ij, x), 
					std::back_inserter(intersections)
				);
				const auto newSegs{convertToSegments1(intersections, x)};
				if (!newSegs.empty()) {
					auto& segmentsInPlace{ arr[x][ij] };
					segmentsInPlace.add(pr, newSegs);
				}

			}
		}
	}
}

void buildGridSlicesSearchMaps(Filler::GridSlices& gS)
{
	log("Simplifying surface slices", 3);
	for (auto& axis : gS) {
		for (auto& slice : axis) {
			slice.second.simplifySurfaces();
		}
	}
	
	log("Building slices triangulations", 3);
	for (auto& axis : gS) {
		for (auto& slice : axis) {
			slice.second.buildTriangulations();
		}
	}

	log("Building slices search maps", 3); 
	std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
		std::execution::par,
#endif
		gS.begin(), gS.end(),
		[&](auto& axis) {
			std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
				std::execution::par,
#endif
				axis.begin(), axis.end(),
				[&](auto& slice) {
					slice.second.buildSearchMap();
					slice.second.cleanSurfaces();
				}
			);
		}
	);
}

void zip(
	const std::vector<Priority>& p,
	const Groups& v,
	const Groups& s,
	std::vector<std::tuple<Priority, Group, Group>>& zipped) 
{
	for (auto i{ 0 }; i < p.size(); i++) {
		zipped.push_back(std::make_tuple(p[i], v[i], s[i]));
	}
}

void unzip(
	const std::vector<std::tuple<Priority, Group, Group>>& zipped,
	std::vector<Priority>& p,
	Groups& v,
	Groups& s)
{
	for (auto i{ 0 }; i < p.size(); i++) {
		p[i] = std::get<0>(zipped[i]);
		v[i] = std::get<1>(zipped[i]);
		s[i] = std::get<2>(zipped[i]);
	}
}

void orderInDecreasingPriority(
	std::vector<Priority>& p,
	Groups& v,
	Groups& s)
{
	std::vector<std::tuple<Priority, Group, Group>> zipped;
	zip(p, v, s, zipped);
	std::sort(std::begin(zipped), std::end(zipped),
		[&](const auto& a, const auto& b)
		{
			return std::get<0>(a) > std::get<0>(b);
		});
	unzip(zipped, p, v, s);

}

void Filler::mergeGroupsWithSamePriority(
	Groups& vGroups,
	Groups& sGroups)
{
	std::map<Priority, std::set<GroupId>> prIdMap;
	for (auto g{ 0 }; g < groupPriorities_.size(); g++) {
		prIdMap[getGroupPriority(g)].insert(g);
	}

	std::map<GroupId, GroupId, std::greater<GroupId> > idToErase;
	for (const auto& [pr, gId] : prIdMap) {
		auto idToKeep{ *gId.begin() };
		for (auto it{ ++gId.begin() }; it != gId.end(); it++) {
			idToErase.emplace(*it, idToKeep);
		}
	}

	for (const auto& [erase, keep] : idToErase) {
		std::copy(vGroups[erase].elements.begin(), vGroups[erase].elements.end(), std::back_inserter(vGroups[keep].elements));
		std::copy(sGroups[erase].elements.begin(), sGroups[erase].elements.end(), std::back_inserter(sGroups[keep].elements));
		vGroups.erase(vGroups.begin() + erase);
		sGroups.erase(sGroups.begin() + erase);
		groupPriorities_.erase(groupPriorities_.begin() + erase);
	}
	log("Ordering groups", 1);
	orderInDecreasingPriority(groupPriorities_, vGroups, sGroups);
	log("Groups ordered", 1);

}

Mesh initializeMeshIfEmpty(const Mesh& lhs, const Mesh& rhs)
{
	if (lhs.emptyOfElements()) {
		Mesh res;
		res.grid = rhs.grid;
		res.groups.resize(rhs.groups.size());
		return res;
	}
	else {
		return lhs;
	}
}

FillerPolyhedrons buildFillerPolyhedrons(
	const Coordinates& vCoords,
	const Elements& vElems,
	const Coordinates& sCoords,
	const Elements& sElems )
{
	FillerPolyhedrons r;
	
	{
		Mesh m;
		m.coordinates = vCoords;
		m.groups = { Group() };
		m.groups[0].elements = vElems;
		Manifolder mf{ m };
		r.volumes  = buildPolyhedronFromMesh(mf.getClosedSurfacesMesh());
		r.surfaces = buildPolyhedronFromMesh(mf.getOpenSurfacesMesh());
	}

	CGAL::copy_face_graph(buildPolyhedronFromElements(sCoords, sElems), r.surfaces);
	Polyhedron pAux{r.volumes};
	reassignFacetsWithPredicate(r.aligned, pAux, isFaceContainedInAnyCartesianPlane);
	reassignFacetsWithPredicate(r.aligned, r.surfaces, isFaceContainedInAnyCartesianPlane);
	Polyhedron trash;
	reassignFacetsWithPredicate(trash, r.volumes, polyhedronTools::isNotValidFace);
	reassignFacetsWithPredicate(trash, r.surfaces, polyhedronTools::isNotValidFace);
	reassignFacetsWithPredicate(trash, r.aligned, polyhedronTools::isNotValidFace);
	if (trash.size_of_facets() != 0) {
		std::cerr << "WARNING: Filler detected " << trash.size_of_facets()
			<< " invalid facets which will be ignored" << std::endl;
		throw std::runtime_error("Invalid areas exist");
	}
	return r; 
}

Filler::Filler(
	const Mesh& volumeMesh,
	const Mesh& surfaceMesh,
	const std::vector<Priority>& groupPriorities)
{
	utils::meshTools::checkNoNullAreasExist(volumeMesh);
	utils::meshTools::checkNoNullAreasExist(surfaceMesh);
	auto vM{ initializeMeshIfEmpty(volumeMesh, surfaceMesh) };
	auto sM{ initializeMeshIfEmpty(surfaceMesh, volumeMesh) };
	assert(sM.grid          == vM.grid);
	assert(sM.groups.size() == vM.groups.size());
	grid_ = vM.grid;

	assert(groupPriorities.size() == vM.groups.size() 
		|| groupPriorities.size() == 0);
	if (groupPriorities.size() != 0) {
		groupPriorities_ = groupPriorities;
	}
	else {
		groupPriorities_.resize(vM.groups.size());
		std::iota(groupPriorities_.begin(), groupPriorities_.end(), 0);
	}
	auto vGroups{ vM.groups };
	auto sGroups{ sM.groups };
	mergeGroupsWithSamePriority(vGroups, sGroups);

	for (std::size_t gId{ 0 }; gId < vGroups.size(); ++gId) {
		std::stringstream ss;
		ss << "Building filler for group " << gId;
		log(ss.str(), 1);
		auto fP{
			buildFillerPolyhedrons(
				vM.coordinates, vGroups[gId].elements,
				sM.coordinates, sGroups[gId].elements)
		};
		const auto pr{ getGroupPriority(gId) };
		
		log("Slicing volumes", 2);
		sliceNonAlignedByGrid(slices_, fP.volumes, grid_, pr, SlicingMode::Volume);
		log("Slicing surfaces", 2);
		sliceNonAlignedByGrid(slices_, fP.surfaces, grid_, pr, SlicingMode::Surface);
		log("Slicing aligned", 2);
		sliceAlignedByGrid(slices_, fP.aligned, grid_, pr);
		log("Building segments arrays", 2);
		buildSegmentsArray(segmentsArray_, fP.aligned, grid_, pr);
		buildSegmentsArray(segmentsArray_, fP.volumes, grid_, pr);
	}
	log("Building slices search maps", 2);
	buildGridSlicesSearchMaps(slices_);
	log("Filling finished");
}

FaceFilling Filler::getFaceFilling(const CellIndex& c) const
{
	//assert(getFillingState(c).partial());
	auto it{ slices_[c.axis].find(c.getSliceNumber()) };
	if (it != slices_[c.axis].end()) {
		return it->second.getFaceFilling(c.getArrayIndex());
	}
	return FaceFilling();
}

EdgeFilling Filler::getEdgeFilling(const CellIndex& c) const 
{
	if (segmentsArray_[c.axis].empty()) {
		return EdgeFilling{};
	}
	auto it{ segmentsArray_[c.axis].find(c.getArrayIndex()) };
	if (it != segmentsArray_[c.axis].end()) {
		return it->second.getEdgeFilling(c.getSliceNumber());
	}
	return EdgeFilling();
}

FillingState Filler::getFillingState(const CellIndex& c) const
{
	auto it{ slices_[c.axis].find(c.getSliceNumber())};
	if (it == slices_[c.axis].end()) {
		return { FillingType::Empty };
	}
	else {
		return it->second.getFillingState(c.getArrayIndex());
	}
}

Elements buildTriangleElements(Coordinates& cs, const TriVs tris) 
{
	Elements r;
	for (const auto& p : tris) {
		Element e;
		e.type = Element::Type::Surface;
		e.vertices.reserve(3);
		for (const auto& v : p) {
			e.vertices.push_back(cs.size());
			cs.push_back(v);
		}
		r.push_back(e);
	}
	return r;
}

Elements buildLineElements(Coordinates& cs, const LinVs lins)
{
	Elements r;
	for (const auto& p : lins) {
		Element e;
		e.type = Element::Type::Line;
		e.vertices.reserve(2);
		for (const auto& v : p) {
			e.vertices.push_back(cs.size());		
			cs.push_back(v);
		}
		r.push_back(e);
	}
	return r;
}

void insertElementsInGroup(Group& g, const Elements& es)
{
	g.elements.insert(
		g.elements.end(),
		es.begin(), es.end()
	);
}

Mesh Filler::getMeshFilling() const
{
	Mesh m;
	m.grid = grid_;
	m.groups.resize(groupPriorities_.size());
	for (auto gId{0}; gId < m.groups.size(); ++gId) {
		for (const auto& x : { X, Y, Z }) {
			for (const auto& [i, slice]: slices_[x]) {
				const Priority pr{ getGroupPriority(gId) };
				insertElementsInGroup(
					m.groups[gId],
					buildTriangleElements(
						m.coordinates,
						slice.buildAllTriVs(pr, x, (Height)i)
					)
				);
				insertElementsInGroup(
					m.groups[gId],
					buildLineElements(
						m.coordinates,
						slice.buildAllLinVs(pr, x, (Height)i)
					)
				);

			}
		}
	}

	return m;
}


}
}
}