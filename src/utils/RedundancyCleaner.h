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
    static void removeDegenerateElements(Mesh&);
    static void removeElementsWithCondition(Mesh&, std::function<bool(const Element&)>);
    static void removeRepeatedElements(Mesh&);
    static void removeRepeatedElementsIgnoringOrientation(Mesh&);
    static void removeOverlappedDimensionZeroElementsAndIdenticalLines(Mesh&);
    static void removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(Mesh&);
    static void removeOverlappedElementsByDimension(Mesh&, const std::vector<Element::Type>&);
    static void removeElements(Mesh&, const std::vector<IdSet>&);
private:
    static Elements findDegenerateElements_(const Group&, const Coordinates&);
  };

}
}

