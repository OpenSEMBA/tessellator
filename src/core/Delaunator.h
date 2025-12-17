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
	std::vector<Element> mesh(const std::vector<Polygon>& constrainingPolygons = std::vector<Polygon>()) const;


private:
	const Coordinates* globalCoordinates_ = nullptr;

	typedef CDT::Triangulation<double>								  Triangulation;
	typedef CDT::V2d<double>                                          Point;
	typedef CDT::VertInd                                              PointId;

	typedef boost::bimap<PointId, CoordinateId> IndexPointToId;

	void checkConstraintsArePlanar(const IdSet& targetVertices) const;
	Triangulation buildCDT(IndexPointToId& pointToId, const IdSet& targetVertices, const Polygons& constrainingPolygons) const;
	Elements convertFromCDT(const Triangulation& cdt, const IndexPointToId& pointToId) const;
};

}
}
