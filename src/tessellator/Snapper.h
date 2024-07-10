#pragma once

#include "types/Mesh.h"
#include "utils/GridTools.h"
#include "SnapperOptions.h"

namespace meshlib {
namespace tessellator {

class Snapper {
public:
	
	Snapper(const Mesh& mesh, const SnapperOptions& opts = SnapperOptions());
	Mesh getMesh() const { return mesh_; };
	
private:
	typedef size_t Component;
	struct Replacement 
	{
		Component position;
		CoordinateId id;
	};

	Mesh mesh_;
	SnapperOptions opts_;

	std::pair<Coordinate, Coordinate> findClosestSolverPoint(
		const Relative& rel,
		const Coordinates& solverPoints,
		const utils::GridTools& gT) const;

	std::pair<Coordinates, std::map<Coordinate, std::set<LinV>>> buildListOfValidSolverPoints() const;


	void snap();
};

}
}