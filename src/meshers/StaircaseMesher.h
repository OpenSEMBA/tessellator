#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"
#include "StaircaseMesherOptions.h"

namespace meshlib::meshers {

class StaircaseMesher : public MesherBase {
public:
	StaircaseMesher(const Mesh& in, int decimalPlacesInCollapser = 4, StaircaseMesherOptions opts = StaircaseMesherOptions());
	virtual ~StaircaseMesher() = default;
	Mesh mesh() const;

private:
	int decimalPlacesInCollapser_;

	Mesh surfaceMesh_;
	StaircaseMesherOptions opts_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;

};

}
