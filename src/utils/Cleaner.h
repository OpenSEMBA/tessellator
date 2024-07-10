#pragma once

#include "../types/Map.h"
#include "../types/Mesh.h"

#include <functional>

namespace meshlib {
namespace utils {

class Cleaner {
public:
    static void clean(Mesh&, Map&);
    static void cleanCoords(Mesh&);
    static void fuseCoords(Mesh&);
    static void removeElementsWithCondition(Mesh&, std::function<bool(const Element&)>);
    static void collapseCoordsInLineDegenerateTriangles(Mesh&, const double& areaThreshold);
    static void removeRepeatedElements(Mesh&);
    static void removeRepeatedElementsIgnoringOrientation(Mesh&);
    static void removeElements(Mesh&, const std::vector<IdSet>&);
private:
    static void cleanElems_(Mesh&, Map&);
    static void cleanCoords_(Mesh&, Map&);
    static void fuseCoords_(Mesh&);

    static Elements findDegenerateElements_(const Group&, const Coordinates&);
  };

}
}

