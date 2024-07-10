#include "types/Mesh.h"

#include "PolyhedronTools.h"

namespace meshlib {
namespace cgal {

class Manifolder {
public:
	Manifolder(const Mesh& m);

	Mesh getSurfacesMesh() const;
	Mesh getClosedSurfacesMesh() const;
	Mesh getOpenSurfacesMesh() const;
		
private:
	std::map<GroupId, Polyhedron> closed_;
	std::map<GroupId, Polyhedron> open_;
	Grid grid_;
	GroupId groupsSize_;

	Mesh buildFromGroupToPolyhedronMap(const std::map<GroupId, Polyhedron>&) const;
};

}
}
