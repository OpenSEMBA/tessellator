#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

#include <CDT.h>

#include <boost/bimap.hpp>

namespace meshlib {
namespace core {

 
class Delaunator {
public:
	typedef std::vector<CoordinateId> Polygon;
	typedef std::vector<Polygon> Polygons;
	
	Delaunator(const Coordinates& globalCoordinates);
	std::vector<Element> mesh(const std::vector<Polygon>& constrainingPolygons = std::vector<Polygon>(), bool ignoreCoplanarity = false) const;


private:
	const Coordinates* globalCoordinates_ = nullptr;

	typedef CDT::Triangulation<double>								  Triangulation;
	typedef CDT::V2d<double>                                          Point;
	typedef CDT::VertInd                                              PointId;

	struct PointLesserComparer {
		bool operator() (const Point& pointA, const Point& pointB) const noexcept {
			if (pointA.x < pointB.x) {
				return true;
			}
			if (pointA.x == pointB.x) {
				return pointA.y < pointB.y;
			}
			return false;
		}
	};	

	typedef boost::bimap<boost::bimaps::set_of<Point, PointLesserComparer>, CoordinateId> PointToId;

	void checkConstraintsArePlanar(const IdSet& targetVertices) const;
	Triangulation buildCDT(PointToId& pointToId, const IdSet& targetVertices, const Polygons& constrainingPolygons) const;
	Elements convertFromCDT(const Triangulation& cdt, const PointToId& pointToId) const;
};

}
}
