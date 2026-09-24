#pragma once

#include "types/Mesh.h"
#include "MesherBase.h"
#include "StaircaseMesherOptions.h"

namespace meshlib::meshers {

class StaircaseMesher : public MesherBase {
public:
	StaircaseMesher(const Mesh& in, StaircaseMesherOptions opts = StaircaseMesherOptions());
	virtual ~StaircaseMesher() = default;
	Mesh mesh() const;
    const StaircaseMesherOptions & getOptions() const { return opts_; }

private:
	Mesh surfaceMesh_;
	Mesh volumeMesh_;
	StaircaseMesherOptions opts_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;
	void process(Mesh&, bool compress) const;
	bool collapse_nodes(Mesh& z_output_mesh,const double tolerance);

};

}
