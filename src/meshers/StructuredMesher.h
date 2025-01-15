#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"

namespace meshlib::meshers {

class StructuredMesher : public MesherBase {
public:
	StructuredMesher(const Mesh& in, int decimalPlacesInCollapser = 4);
	virtual ~StructuredMesher() = default;
	Mesh mesh() const;

private:
	int decimalPlacesInCollapser_;

	Mesh surfaceMesh_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;

};

}
