#pragma once

#include "types/Mesh.h"

#include "utils/GridTools.h"
#include "utils/Types.h"

#include "Slice.h"
#include "SegmentsArray.h"

namespace meshlib {
namespace tessellator {
namespace filler {

class Filler {
public:
	using Slices = std::map<SliceNumber, Slice>;
	using GridSlices = std::array<Slices, 3>;
	using SegmentsArray = std::map<ArrayIndex, Segments>;
	using GridSegmentsArray = std::array<SegmentsArray, 3>;

	Filler(
		const Mesh& volumeMesh, 
		const Mesh& surfaceMesh = Mesh(),
		const std::vector<Priority>& groupPriorities = std::vector<Priority>());
	Filler(const Filler&) = delete;
	Filler(Filler&&) = default;
	Filler& operator=(const Filler&) = delete;
	Filler& operator=(Filler&&) = default;
	~Filler() = default;

	EdgeFilling getEdgeFilling(const CellIndex&) const;
	FaceFilling getFaceFilling(const CellIndex&) const;

	FillingState getFillingState(const CellIndex&) const;
	
	Mesh getMeshFilling() const;

private:
	GridSlices slices_;
	GridSegmentsArray segmentsArray_;
	Grid grid_;
	std::vector<Priority> groupPriorities_;
	Priority getGroupPriority(const GroupId& gId) const;

	void mergeGroupsWithSamePriority(Groups& vGroups, Groups& sGroups);

};

}
}
}
