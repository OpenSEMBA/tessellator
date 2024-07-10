#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

#include <boost/bimap.hpp>

namespace meshlib {
namespace cgal {

 
class Delaunator {
public:
	typedef std::vector<CoordinateId> Polygon;
	typedef std::vector<Polygon> Polygons;
	
	Delaunator(const Coordinates* globalCoordinates, const ElementsView& elems = ElementsView());

	std::vector<Element> mesh(
		const IdSet& vertexIds,
		const std::vector<Polygon>& constrainingPolygons = std::vector<Polygon>()
	) const;


private:
	const Coordinates* globalCoords_ = nullptr;
	ElementsView elements_;
	static const double COPLANARITY_ANGLE_TOLERANCE;
	
	struct FaceInfo2
	{
		FaceInfo2() {}
		int nesting_level;
		bool in_domain() {
			return nesting_level % 2 == 1;
		}
	};


	typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
	typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
	typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K>   Fbb;
	typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>       Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              TDS;
	typedef CGAL::Exact_predicates_tag                                Itag;
	typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
	typedef CDT::Point                                                Point;
	typedef CGAL::Polygon_2<K>                                        Polygon_2;
	typedef CDT::Face_handle                                          Face_handle;

	typedef boost::bimap<CDT::Point, CoordinateId> IndexPointToId;

	IndexPointToId buildPointsInIndex(const IdSet& inIds, const Polygons& constraint) const;
	static CDT buildCDT(
		const IndexPointToId& pointToId,
		const IdSet& inIds,
		const Polygons& constrainingPolygons);
	std::vector<Element> convertFromCDT(const CDT& cdt, const IndexPointToId& pointToId) const;
	
	static void mark_domains(CDT& cdt);
	static void mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border);
	void checkIdsAreInRange(
		const IdSet& inIds,
		const std::vector<Polygon>& constrainingPolygons) const;
	static IdSet filterIdsByConstraints(const IdSet& ids, const Polygons&);
};

}
}
