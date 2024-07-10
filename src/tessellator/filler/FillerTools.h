#pragma once

#include "types/CellIndex.h"
#include "cgal/PolyhedronTools.h"
#include "cgal/HPolygonSet.h"
#include "cgal/Tools.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

namespace meshlib {
namespace tessellator {
namespace filler {

using Height = cgal::KType;

struct FaceInfo2
{
	FaceInfo2() {}
	int nesting_level;
	bool in_domain() {
		return nesting_level % 2 == 1;
	}
};

using Itag = CGAL::Exact_predicates_tag;
using Vb = CGAL::Triangulation_vertex_base_2<cgal::K>;
using Fbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, cgal::K>;
using Fb = CGAL::Constrained_triangulation_face_base_2<cgal::K, Fbb>;
using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<cgal::K, TDS, Itag>;
using CDTs = std::vector<CDT>;


bool isCellCrossedByTriangle(const cgal::Triangle2&, const ArrayIndex&);
cgal::Rectangle2 buildCellFace(const ArrayIndex&);
cgal::Polygon buildCellFacePolygon(const ArrayIndex&);

bool isValidFace(const CDT::Face& f);

cgal::HPolygonSet buildPolygonSetFromCDT(
	const std::vector<const CDT::Face*>& tris);

CDT buildCDTFromPolygonWH(const cgal::PolygonWH&);
CDTs buildCDTsFromPolygonSet(const cgal::HPolygonSet&);

cgal::HPolygonSet buildPolygonSetFromPolyhedron(
	const cgal::Polyhedron& p, const Axis& x);


}
}
}