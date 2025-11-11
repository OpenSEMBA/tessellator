#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"

namespace meshlib::meshers {

class StaircaseMesher : public MesherBase {
public:
	StaircaseMesher(const Mesh& in, int decimalPlacesInCollapser = 4);
	virtual ~StaircaseMesher() = default;
	Mesh mesh() const;

private:
	int decimalPlacesInCollapser_;

	Mesh surfaceMesh_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;

};

}
