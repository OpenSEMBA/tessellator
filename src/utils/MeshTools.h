#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"


namespace meshlib::utils::meshTools {

Mesh duplicateCoordinatesUsedByDifferentGroups(const Mesh& mesh);
Mesh duplicateCoordinatesSharedBySingleTrianglesVertex(const Mesh& mesh);

static bool isNode(const Element& e) { return e.isNode(); }
static bool isNotNode(const Element& e) { return !e.isNode(); }
static bool isLine(const Element& e) { return e.isLine(); }
static bool isNotLine(const Element& e) { return !e.isLine(); }
static bool isTriangle(const Element& e) { return e.isTriangle(); }
static bool isNotTriangle(const Element& e) { return !e.isTriangle(); }
static bool isQuad(const Element& e) { return e.isQuad(); }
static bool isNotQuad(const Element& e) { return !e.isQuad(); }
static bool isTetrahedron(const Element& e) { return e.isTetrahedron(); }
static bool isNotTetrahedron(const Element& e) { return !e.isTetrahedron(); }

std::size_t countMeshElementsIf(const Mesh& mesh, std::function<bool(const Element&)> countFilter);
std::vector<Element::Type> getHighestDimensionByGroup(const Mesh& mesh);

Mesh buildMeshFilteringElements(
	const Mesh& in, std::function<bool(const Element&)> filter);
Mesh buildMeshFromSelectedCells(const Mesh& in, const std::set<Cell>& selectedCells);
Mesh buildMeshFromContours(const Mesh&);

std::pair<Coordinate, Coordinate> getElementsBoundingBox(const Mesh&);

Grid getEnlargedGridIncludingAllElements(const Mesh&);
void reduceGrid(Mesh&, const Grid&);
Mesh reduceGrid(const Mesh& m, const Grid& g);

void convertToAbsoluteCoordinates(Mesh&);
	
void checkSlicedMeshInvariants(Mesh& m);
	
void checkNoCellsAreCrossed(const Mesh& m);
void checkNoOverlaps(const Mesh& m);
void checkNoNullAreasExist(const Mesh& m);

std::string info(const Element& e, const Mesh& m);

void mergeGroup(Group& lG, const Group& rG, const CoordinateId& coordCount = 0);
void mergeMesh(Mesh& lMesh, const Mesh& iMesh);
void mergeMeshAsNewGroup(Mesh& lMesh, const Mesh& iMesh);

bool isAClosedTopology(const Elements& es);

}