#pragma once

#include "types/Mesh.h"
#include "utils/GridTools.h"
#include "SnapperOptions.h"

namespace meshlib {
namespace core {

class Snapper {
public:
	enum class SurfaceInversionPolicy {
		Reject,
		AllowDuringCollapse,
	};
	
	Snapper(
		const Mesh& mesh,
		const SnapperOptions& opts = SnapperOptions(),
		SurfaceInversionPolicy surfaceInversionPolicy =
			SurfaceInversionPolicy::Reject);
	Mesh getMesh() const { return mesh_; };
	const std::set<Cell>& getCellsToStructure() const { return cellsToStructure_; }
	
private:
	typedef size_t Component;
	struct Replacement 
	{
		Component position;
		CoordinateId id;
	};

	Mesh mesh_;
	SnapperOptions opts_;
	SurfaceInversionPolicy surfaceInversionPolicy_;
	std::set<Cell> cellsToStructure_;

	std::pair<Coordinate, Coordinate> findClosestSolverPoint(
		const Relative& rel,
		const Coordinates& solverPoints,
		const utils::GridTools& gT) const;

	std::pair<Coordinates, std::map<Coordinate, std::set<LinV>>> buildListOfValidSolverPoints() const;


	void snap();
	void rollbackUnsafeSurfaceSnaps(const Coordinates& originalCoordinates);
};

}
}
