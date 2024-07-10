#pragma once

#include "utils/Types.h"
#include "Types.h"

#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

namespace meshlib {
namespace cgal {

namespace PMP = CGAL::Polygon_mesh_processing;

using Polyhedron = CGAL::Polyhedron_3<K>;

namespace polyhedronTools {

typedef std::array<K::FT, 3> ComparablePoint;
struct Array_traits
{
	struct Equal_3
	{
		bool operator()(const ComparablePoint& p, const ComparablePoint& q) const {
			return (p == q);
		}
	};
	struct Less_xyz_3
	{
		bool operator()(const ComparablePoint& p, const ComparablePoint& q) const {
			return std::lexicographical_compare(p.begin(), p.end(), q.begin(), q.end());
		}
	};
	Equal_3 equal_3_object() const { return Equal_3(); }
	Less_xyz_3 less_xyz_3_object() const { return Less_xyz_3(); }
};

std::pair<Polyhedron, Polyhedron> buildClosedAndOpenPolyhedrons(const Coordinates&, const Elements&);

Elements buildElementsFromPolyhedron(Coordinates&, const Polyhedron&);
Group buildGroupFromPolyhedron(Mesh&, const Polyhedron&);

Mesh buildMeshFromPolyhedron(const Polyhedron&);

Polyhedron buildPolyhedronFromMesh(const Mesh&);
Polyhedron buildPolyhedronFromElements(const Coordinates& cs, const Elements& es);
Polyhedron buildPolyhedronFromSoup(
	std::vector<ComparablePoint>& ps,
	std::vector<std::vector<std::size_t> >& tIds);

void reassignFacetsWithPredicate(
	Polyhedron& tm, Polyhedron& sm,
	std::function<bool(const Polyhedron::Facet&)> predicate);

void appendToPointsAndIds(
	std::vector<ComparablePoint>&,
	std::vector<std::vector<std::size_t> >&,
	const Polyhedron::Facet&);

void exportMeshToSTL(const Mesh&, const std::string outName);

bool isValidFace(const Polyhedron::Face&);
bool isNotValidFace(const Polyhedron::Face&);
std::pair<Axis, CellDir> getFaceCartesianPlane(const Polyhedron::Facet&);
bool isFaceContainedInAnyCartesianPlane(const Polyhedron::Facet&);


}
}
}