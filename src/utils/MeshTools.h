#pragma once

#include "types/Mesh.h"

namespace meshlib {
namespace utils {
namespace meshTools {

Mesh duplicateCoordinatesUsedByDifferentGroups(const Mesh& mesh);

static bool isTriangle(const Element& e) { return e.isTriangle(); }
static bool isNotTriangle(const Element& e) { return !e.isTriangle(); }
static bool isTetrahedron(const Element& e) { return e.isTetrahedron(); }
static bool isNotTetrahedron(const Element& e) { return !e.isTetrahedron(); }

Mesh buildMeshFilteringElements(
	const Mesh& in, std::function<bool(const Element&)> filter);

Grid getEnlargedGridIncludingAllElements(const Mesh&);

std::pair<VecD, VecD> getBoundingBox(const Mesh&);

void reduceGrid(Mesh&, const Grid&);
Mesh reduceGrid(const Mesh& m, const Grid& g);

Mesh setGrid(const Mesh& m, const Grid& g);

void convertToAbsoluteCoordinates(Mesh&);
	
void checkSlicedMeshInvariants(Mesh& m);
	
void checkNoCellsAreCrossed(const Mesh& m);
void checkNoOverlaps(const Mesh& m);
void checkNoNullAreasExist(const Mesh& m);

std::string info(const Element& e, const Mesh& m);

void mergeGroup(Group& lG, const Group& rG, const CoordinateId& coordCount = 0);
void mergeMesh(Mesh& lMesh, const Mesh& iMesh);
void mergeMeshAsNewGroup(Mesh& lMesh, const Mesh& iMesh);

}
}
}