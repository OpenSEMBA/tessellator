#include "PolyhedronTools.h"

#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/manifoldness.h>

#include <CGAL/boost/graph/IO/STL.h>

#include <utils/GridTools.h>

namespace meshlib {
namespace cgal {
namespace polyhedronTools {


using PolyhedronWithId = CGAL::Polyhedron_3 < K, CGAL::Polyhedron_items_with_id_3>;


Polyhedron buildPolyhedronFromElements(const Coordinates& cs, const Elements& es)
{
	std::vector<ComparablePoint> points(cs.size());
	for (auto const& c : cs) {
		points[&c - &cs.front()] =
			CGAL::make_array<K::FT>(c[X], c[Y], c[Z]);
	}

	std::vector<std::vector<std::size_t> > polygons;
	polygons.reserve(es.size());
	for (auto const& e : es) {
		if (e.isTriangle()) {
			polygons.push_back(e.vertices);
		}
	}
	return buildPolyhedronFromSoup(points, polygons);
}

std::pair<Polyhedron, Polyhedron> buildClosedAndOpenPolyhedrons(const Coordinates& cs, const Elements& es)
{
	Polyhedron closedP, openP;
	auto aux{ buildPolyhedronFromElements(cs, es) };
	PMP::duplicate_non_manifold_vertices(aux);
	std::vector<Polyhedron> ccPolys;
	PMP::split_connected_components(aux, ccPolys);
	for (auto& p : ccPolys) {
		if (p.is_closed()) {
			CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(p);
			CGAL::copy_face_graph(p, closedP);
		}
		else {
			CGAL::copy_face_graph(p, openP);
		}
	}

	return std::make_pair(closedP, openP);
}

Elements buildElementsFromPolyhedron(Coordinates& cs, const Polyhedron& polyIn)
{
	PolyhedronWithId poly;
	CGAL::copy_face_graph(polyIn, poly);
	std::size_t id = 0;
	for (auto v = poly.vertices_begin(); v != poly.vertices_end(); v++) {
		v->id() = id++;
	}

	std::size_t previousCoordSize = cs.size();
	cs.reserve(cs.size() + poly.size_of_vertices());
	for (auto v = poly.vertices_begin(); v != poly.vertices_end(); v++) {
		const auto& p{ v->point() };
		cs.push_back(Coordinate({ p.x(), p.y(), p.z() }));
	}

	Elements res(poly.size_of_facets());
	ElementId eId = 0;
	for (auto f = poly.facets_begin(); f != poly.facets_end(); ++f) {
		CoordinateIds vIds;
		auto v = f->facet_begin();
		do {
			std::size_t id = v->vertex()->id() + previousCoordSize;
			vIds.push_back(id);
		} while (++v != f->facet_begin());
		res[eId] = Element(vIds, Element::Type::Surface);
		eId++;
	}
	return res;
}

Group buildGroupFromPolyhedron(Mesh& m, const Polyhedron& p)
{
	Group res;
	if (!CGAL::is_empty(p)) {
		res.elements = buildElementsFromPolyhedron(m.coordinates, p);
	}
	return res;
}

Mesh buildMeshFromPolyhedron(const Polyhedron& p) 
{
	Mesh m;
	m.groups = { buildGroupFromPolyhedron(m, p)};
	return m;
}

Polyhedron buildPolyhedronFromMesh(const Mesh& m)
{
	Polyhedron p;
	for (const auto& g : m.groups) {
		Polyhedron cP, oP;
		std::tie(cP, oP) = buildClosedAndOpenPolyhedrons(m.coordinates, g.elements);
		CGAL::copy_face_graph(cP, p);
		CGAL::copy_face_graph(oP, p);
	}
	return p;
}

void appendToPointsAndIds(
	std::vector<ComparablePoint>& ps, 
	std::vector<std::vector<std::size_t> >& tIds, 
	const Polyhedron::Facet& f)
{
	std::vector<std::size_t> triId;
	auto vIt = f.facet_begin();
	do {
		triId.push_back(ps.size());
		ps.push_back(
			CGAL::make_array<K::FT>(
				vIt->vertex()->point().x(),
				vIt->vertex()->point().y(), 
				vIt->vertex()->point().z()
			)
		);
	} while (++vIt != f.facet_begin());
	tIds.push_back(triId);
}

Polyhedron buildPolyhedronFromSoup(
	std::vector<ComparablePoint>& points,
	std::vector<std::vector<std::size_t> >& polygons)
{
	Polyhedron res;
	PMP::orient_polygon_soup(points, polygons);
	PMP::polygon_soup_to_polygon_mesh(points, polygons, res);
	return res;
}

void reassignFacetsWithPredicate(
	Polyhedron& tm, Polyhedron& sm, 
	std::function<bool(const Polyhedron::Facet&)> predicate)
{
	typedef CGAL::dynamic_face_property_t<bool> Face_property_tag;
	
	auto passesPredicateMap{ get(Face_property_tag(), sm) };
	auto noPassesPredicateMap{ get(Face_property_tag(), sm) };
	for (auto f{ sm.facets_begin() }; f != sm.facets_end(); ++f) {
		if (predicate(*f)) {
			put(passesPredicateMap, f, true);
		}
		else {
			put(noPassesPredicateMap, f, true);
		}
	}
	
	using FaceFilteredGraph = CGAL::Face_filtered_graph<Polyhedron>;
	{
		FaceFilteredGraph filtered(sm, true, passesPredicateMap);
		CGAL::copy_face_graph(filtered, tm);
	}
	{
		FaceFilteredGraph filtered(sm, true, noPassesPredicateMap);
		Polyhedron aux;
		CGAL::copy_face_graph(filtered, aux);
		std::swap(aux, sm);
	}
}

void exportMeshToSTL(const Mesh& m, const std::string outName)
{
	std::cout << "[STLExport] -- Exporting mesh to STL" << std::endl;

	utils::GridTools gT(m.grid);
	Coordinates pos;
	for (const auto& rel : m.coordinates) {
		pos.push_back(gT.getPos(rel));
	}
	
	bool exp{ CGAL::IO::write_STL(
		outName + ".stl", 
		buildPolyhedronFromMesh(Mesh{m.grid, pos, m.groups}))
	};
	if (!exp) {
		throw std::runtime_error("STL export failed");
	}

}

GridPlane getFaceCartesianPlane(const Polyhedron::Facet& f)
{
	assert(isFaceContainedInAnyCartesianPlane(f));

	for (const auto& x : { X, Y, Z }) {
		std::vector<double> cC;
		cC.reserve(f.size());
		auto vIt = f.facet_begin();
		do {
			cC.push_back(vIt->vertex()->point()[int(x)]);
		} while (++vIt != f.facet_begin());
		if (std::equal(cC.begin() + 1, cC.end(), cC.begin())) {
			return std::make_pair(x, cC.front());
		}
	}

	throw std::runtime_error("Unable to find cartesian plane");
}

bool isValidFace(const Polyhedron::Face& f)
{
	auto v{ f.facet_begin() };
	do {
		auto p{ v->vertex()->point() };
		auto pN{ std::next(v)->vertex()->point() };
		if (p == pN) {
			return false;
		}
	} while (++v != f.facet_begin());
	return true;
}

bool isNotValidFace(const Polyhedron::Face& f) 
{
	return !isValidFace(f);
}

bool isFaceContainedInAnyCartesianPlane(const Polyhedron::Facet& f)
{
	for (const auto& x : { 0, 1, 2 }) {
		std::vector<double> cC;
		cC.reserve(f.size());
		auto vIt = f.facet_begin();
		do {
			cC.push_back(vIt->vertex()->point()[int(x)]);
		} while (++vIt != f.facet_begin());
		if (std::equal(cC.begin() + 1, cC.end(), cC.begin()) 
			&& std::floor(cC[0]) == cC[0]) {
			return true;
		}
	}
	return false;
}

}
}
}
