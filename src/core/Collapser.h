#pragma once

#include "types/Mesh.h"

namespace meshlib {
namespace core {

class Collapser {
public:
	Collapser(const Mesh&, int decimalPlaces, const std::vector<Element::Type>& dimensionPolicy = {});

	Mesh getMesh() const { return mesh_; }

private:
	Mesh mesh_;
	std::vector<Element::Type> dimensionPolicy_;

	void collapseDegenerateElements(Mesh& m, const double& areaThreshold);
};

}
}