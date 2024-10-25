#pragma once

#include "types/Mesh.h"

namespace meshlib {
namespace tessellator {

class Collapser {
public:
	Collapser(const Mesh&, int decimalPlaces);

	Mesh getMesh() const { return mesh_; }

private:
	Mesh mesh_;
};

}
}