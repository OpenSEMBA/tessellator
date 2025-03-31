#pragma once

#include "types/Mesh.h"

namespace meshlib {
namespace core {

class Collapser {
public:
	Collapser(const Mesh&, int decimalPlaces);

	Mesh getMesh() const { return mesh_; }

private:
	Mesh mesh_;
	void collapseDegenerateElements(Mesh& m, const double& areaThreshold);
};

}
}