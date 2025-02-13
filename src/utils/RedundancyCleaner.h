#pragma once

#include "../types/Mesh.h"
#include "Types.h"

#include <functional>

namespace meshlib {
namespace utils {

class RedundancyCleaner {
public:
    static void cleanCoords(Mesh&);
    static void fuseCoords(Mesh&);
    static void removeElementsWithCondition(Mesh&, std::function<bool(const Element&)>);
    static void collapseCoordsInLineDegenerateTriangles(Mesh&, const double& areaThreshold);
    static void removeRepeatedElements(Mesh&);
    static void removeRepeatedElementsIgnoringOrientation(Mesh&);
    static void removeOverlappedElementsForLineMeshing(Mesh&);
    static void removeOverlappedElementsForSurfaceMeshing(Mesh&);
    static void removeElements(Mesh&, const std::vector<IdSet>&);
private:
    static void fuseCoords_(Mesh&);

    static Elements findDegenerateElements_(const Group&, const Coordinates&);
  };

}
}

