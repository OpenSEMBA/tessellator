#pragma once

#include "../types/Mesh.h"
#include "Types.h"

#include <functional>

namespace meshlib::utils::redundancyCleaner {

void cleanCoords(Mesh& mesh);
void fuseCoords(Mesh& mesh);

void removeDegenerateElements(Mesh& mesh);

void removeElementsWithCondition(Mesh& mesh, std::function<bool(const Element&)> condition);
void removeRepeatedElements(Mesh& mesh);
void removeRepeatedElementsIgnoringOrientation(Mesh& mesh);

void removeOverlappedDimensionZeroElementsAndIdenticalLines(Mesh& mesh);
void removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(Mesh& mesh);
void removeOverlappedElementsByDimension(Mesh& mesh, const std::vector<Element::Type>& dimension);

void removeElements(Mesh& mesh, const std::vector<IdSet>& toRemove);

}

