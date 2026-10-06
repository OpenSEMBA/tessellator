#pragma once

#include <string>
#include <vector>

#include "types/Mesh.h"
#include "MesherBase.h"
#include "StaircaseMesherOptions.h"

namespace meshlib::meshers {

struct MesherPhaseTiming {
    std::string phase;
    double seconds = 0.0;
};

class StaircaseMesher : public MesherBase {
public:
	StaircaseMesher(const Mesh& in, StaircaseMesherOptions opts = StaircaseMesherOptions());
	virtual ~StaircaseMesher() = default;
	Mesh mesh() const;
    const StaircaseMesherOptions & getOptions() const { return opts_; }

    // Wall-clock duration of each processing phase, in execution order.
    const std::vector<MesherPhaseTiming> & getTimings() const { return timings_; }

private:
	Mesh surfaceMesh_;
	Mesh volumeMesh_;
	StaircaseMesherOptions opts_;
	mutable std::vector<MesherPhaseTiming> timings_;

	virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
	void process(Mesh&) const;
	void process(Mesh&, bool compress) const;
	void process(Mesh&, bool compress, const std::string& label) const;
	void addTiming(const std::string& phase, double seconds) const;
	bool collapse_nodes(Mesh& z_output_mesh,const double tolerance);

};

}
