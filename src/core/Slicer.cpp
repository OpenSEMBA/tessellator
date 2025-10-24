#include "Slicer.h"

#include "utils/Geometry.h"
#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"
#include "utils/ConvexHull.h"
#include "core/Collapser.h"

#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif

namespace meshlib {
namespace core {
using namespace utils;


void orient(const Coordinates& coords,
    std::vector<Element>& elems,
    const TriV& tri) {
    Coordinate uNormal = (tri[1] - tri[0]) ^ (tri[2] - tri[0]);
    for (std::size_t e = 0; e < elems.size(); e++) {
        if (elems[e].type == Element::Type::Surface) {
            std::array<Coordinate, 3> elemCoords;
            for (std::size_t i = 0; i < 3; i++) {
                elemCoords[i] = coords[elems[e].vertices[i]];
            }
            Coordinate sNormal = (elemCoords[1] - elemCoords[0]) ^
                (elemCoords[2] - elemCoords[0]);
            if (sNormal * uNormal < 0.0) {
                if (elems[e].vertices.size() == 3) {
                    std::swap(elems[e].vertices[1], elems[e].vertices[2]);
                    continue;
                }
                std::swap(elems[e].vertices[1], elems[e].vertices[3]);
            }
        }
    }
}


Slicer::Slicer(const Mesh& input, const std::vector<Element::Type>& dimensionPolicy, const SlicerOptions& opts) :
    GridTools(input.grid),
    opts_(opts)
{
    // Ensures that all coordinates have a fixed number of decimal places.
    Mesh collapsed = input;
    collapsed.coordinates = absoluteToRelative(collapsed.coordinates);
    collapsed = Collapser{ collapsed, opts_.initialCollapsingDecimalPlaces, dimensionPolicy }.getMesh();
    collapsed.coordinates = relativeToAbsolute(collapsed.coordinates);

    // Slices.
    mesh_.grid = collapsed.grid;
    mesh_.coordinates.reserve(mesh_.coordinates.size() * 100);
    mesh_.groups.resize(collapsed.groups.size());

    for (auto& g : mesh_.groups) {
        g.elements.reserve(g.elements.size() * 100);
    }
    Coordinates& sCoords = mesh_.coordinates;

    for (std::size_t g = 0; g < collapsed.groups.size(); g++) {
        std::for_each(
#ifdef TESSELLATOR_EXECUTION_POLICIES
            std::execution::seq,
#endif
            collapsed.groups[g].elements.begin(), collapsed.groups[g].elements.end(),
            [&](auto const& e) {
                Elements elements;
                if (e.type == Element::Type::Surface) {
                    TriV triV{ Geometry::asTriV(e, collapsed.coordinates) };

                    elements = { sliceTriangle(sCoords, triV) };
                    orient(sCoords, elements, triV);
                }
                else if (e.isLine()) {
                    LinV lineV{ Geometry::asLinV(e, collapsed.coordinates) };

                    elements = { sliceLine(sCoords, lineV) };
                }
                else if (e.isNode()) {
                    auto& coordinate = collapsed.coordinates[e.vertices[0]];
                    sCoords.push_back(getRelative(coordinate));
                    elements = { e };
                }
                else {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock{ writingElements_ };
                    for (const auto& e: elements) {
                        mesh_.groups[g].elements.push_back(e);
                    }
                }
            }
        );
    }

    RedundancyCleaner::removeElementsWithCondition(mesh_, [](auto e) {return !(e.isTriangle() || e.isLine() || e.isNode()); });
    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::removeDegenerateElements(mesh_);

    // Checks ensured post conditions.
    meshTools::checkNoCellsAreCrossed(mesh_);
    meshTools::checkNoNullAreasExist(mesh_);
}

Elements Slicer::sliceTriangle(
        Coordinates& sCoords,
        const TriV& tri)
{
    Elements res;
    for (auto const& itCell : 
            buildCellCoordIdMap(sCoords, buildIntersectionsWithGridPlanes<3>(sCoords, tri))) {
        const IdSet& vIds = itCell.second;
        if (vIds.size() < 3) {
            continue;
        }

        Coordinates coords;
        for (auto const& id : vIds) {
            coords.push_back(sCoords[id]);
        }
        if (Geometry::areCollinear(coords)) {
            continue;
        }

        auto n{ utils::Geometry::normal(tri) };
        auto path = utils::ConvexHull(&sCoords).get(vIds, n);
        Elements newTris = buildTrianglesFromPath(sCoords, path);
        res.insert(res.end(), newTris.begin(), newTris.end());
    }
    return res;
}

Elements Slicer::sliceLine(Coordinates& relativeCoordinates, const LinV& line) {
    Elements newLines;
    auto newIds = buildIntersectionsWithGridPlanes<2>(relativeCoordinates, line);
    auto firstIdIt = newIds.begin();
    auto secondIdIt = newIds.begin();
    ++secondIdIt;
    while (secondIdIt != newIds.end()) {
        newLines.push_back(Element({ *firstIdIt, *secondIdIt }, Element::Type::Line));
        ++firstIdIt;
        ++secondIdIt;
    }

    return newLines;
}

template<std::size_t N>
IdSet Slicer::buildIntersectionsWithGridPlanes(
    Coordinates& sCoords,
    const ElemV<N>& element)
{
    Coordinates newCoordinates;
    std::set<Coordinate> auxCoordinatesSet;
    for (const auto& v : element) {
        auxCoordinatesSet.insert(getRelative(v));
    }

    std::vector<std::pair<Plane, LinV>> edgeIntersectionsWithPlanes;

    if (element.size() == 2) {
        LinV line;
        line[0] = element[0];
        line[1] = element[1];
        auto polyline = meshSegments(line);
        for (const auto& v : polyline) {
            newCoordinates.push_back(v);
        }

    }
    else if (element.size() == 3) {
        TriV triangle;
        triangle[0] = element[0];
        triangle[1] = element[1];
        triangle[2] = element[2];
        edgeIntersectionsWithPlanes = getEdgeIntersectionsWithPlanes(triangle);
        std::set<Plane> processedPlanes;

        for (const auto& intersection : edgeIntersectionsWithPlanes) {
            for (const auto& v : meshSegments(intersection.second)) {
                bool inProcessedPlanes = false;
                std::size_t equalAxes = 0;
                for (Axis axis = X; axis <= Z; ++axis) {
                    if (approxDir(toNearestVertexDir(v[axis]), v[axis])) {
                        Plane expectedPlane = std::make_pair(v[axis], axis);
                        ++equalAxes;

                        if (expectedPlane != intersection.first){
                            inProcessedPlanes = inProcessedPlanes || processedPlanes.count(expectedPlane) != 0;
                        }
                    }
                }
                if (equalAxes < 2 || !inProcessedPlanes){
                    auxCoordinatesSet.insert(v);
                }
            }
            processedPlanes.insert(intersection.first);
        }
        newCoordinates.insert(newCoordinates.end(), auxCoordinatesSet.begin(), auxCoordinatesSet.end());
    }

    CoordinateId previousNumberOfCoords;
    {
        std::lock_guard<std::mutex> lock{ writingCoordinates_ };
        previousNumberOfCoords = sCoords.size();
        sCoords.insert(sCoords.end(), newCoordinates.begin(), newCoordinates.end());
    }
    IdSet res;
    for (CoordinateId i{ previousNumberOfCoords }; i < sCoords.size(); ++i) {
        res.insert(res.end(), i);
    }
    return res;
}

Slicer::CellCoordIdMap Slicer::buildCellCoordIdMap(
    Coordinates& sCoords,
    const IdSet& idSet) const
{
    CellCoordIdMap cells;
    for (const auto& i : idSet) {
        for (auto const& cell : getTouchingCells(sCoords[i])) {
            cells[cell].insert(i);
        }
    }
    
    CellCoordIdMap res;
    {
        std::map<IdSet, Cell> rCells;
        for (auto const& c : cells) {
            rCells.emplace(IdSet(c.second.begin(), c.second.end()), c.first);
        }
        for (auto const& s : rCells) {
            res.emplace(s.second, IdSet(s.first.begin(), s.first.end()));
        }
    }

    return res;
}

Elements Slicer::buildTrianglesFromPath(
    const Coordinates& coords, 
    const std::vector<CoordinateId>& path)
{
    Elements tris;
    
    if (path.size() < 3) {
        return tris;
    }
 
    auto p{ path };
    {
        std::size_t turns = 0;
        while (
            Geometry::isDegenerate(
                Geometry::asTriV(Element({ p[0],p[1],p[2] }), coords)
            )
            ) {
            std::rotate(p.begin(), p.begin() + 1, p.end());
            if (turns == p.size() - 1) {
                throw std::runtime_error("All points are collinear.");
            }
            turns++;
        }
    }

    Element tri;
    tri.type = Element::Type::Surface;
    tri.vertices.resize(3);
    tri.vertices[0] = p[0];
    for (std::size_t i = 1; i < p.size()-1; i++) {
        tri.vertices[1] = p[i];
        tri.vertices[2] = p[i + 1];

        tris.push_back(tri);
    }
    return tris;
}

Slicer::PolylineV meshSegment(const Coordinate& relPrev, const Coordinate& relNext)
{
    Slicer::PolylineV res;
    res.push_back(relPrev);
    if (relPrev == relNext) {
        return res;
    }
    if (GridTools::approx(relPrev, relNext)) {
        return res;
    }
    res.push_back(relNext);
    return res;
}

Slicer::PolylineV Slicer::meshSegments(const LinV& lineCoords) const {

    Coordinate  posIni, posEnd;
    Cell       cellIni, cellEnd;
    Coordinate  posPrev, posNext;
    Coordinate  relPrev, relNext;
    Cell       cellPrev, cellNext;
    posIni = getPos(getRelative(lineCoords[0]));
    posEnd = getPos(getRelative(lineCoords[1]));
    cellIni = getCell(posIni);
    cellEnd = getCell(posEnd);
    for (std::size_t d = 0; d < 3; d++) {
        if ((cellIni(d) < cellEnd(d)) &&
            (posEnd(d) == getPosDir(cellEnd(d), d))) {
            cellEnd(d)--;
        }
        if ((cellIni(d) > cellEnd(d)) &&
            (posIni(d) == getPosDir(cellIni(d), d))) {
            cellIni(d)--;
        }
    }

    PolylineV res;
    posNext = posIni;
    cellNext = cellIni;
    do {
        posPrev = posNext;
        cellPrev = cellNext;
        getCellPosNext(cellNext, posNext, cellPrev, posPrev, cellEnd, posEnd);
        relPrev = getRelative(posPrev, cellPrev);
        relNext = getRelative(posNext, cellNext);
        for (std::size_t d = 0; d < 3; d++) {
            if (approxDir(toNearestVertexDir(relPrev[d]), relPrev[d])) {
                relPrev[d] = toNearestVertexDir(relPrev[d]);
            }
            if (approxDir(toNearestVertexDir(relNext[d]), relNext[d])) {
                relNext[d] = toNearestVertexDir(relNext[d]);
            }
        }
        PolylineV aux = meshSegment(relPrev, relNext);
        if (!aux.empty()) {
            if (res.empty()) {
                res.insert(res.end(), aux.begin(), aux.end());
            }
            else {
                res.insert(res.end(), aux.begin() + 1, aux.end());
            }
        }
    } while (cellPrev != cellNext);
    return res;
}

void Slicer::getCellPosNext(
    Cell& cellNext, Coordinate& posNext,
    const Cell& cellPrev, const Coordinate& posPrev,
    const Cell& cellEnd, const Coordinate& posEnd) const {
    std::array<CoordinateDir, 3> t;
    Cell n;
    Coordinate p;

    for (std::size_t d = 0; d < 3; d++) {
        if (cellPrev(d) == cellEnd(d)) {
            n[d] = cellEnd(d);
            p[d] = posEnd(d);
            t[d] = 1.0;
        }
        else if (cellPrev(d) < cellEnd(d)) {
            n[d] = cellPrev(d) + 1;
            p[d] = getPosDir(n[d], d);
            t[d] = (p[d] - posPrev(d)) / (posEnd(d) - posPrev(d));
        }
        else {
            n[d] = cellPrev(d) - 1;
            p[d] = getPosDir(n[d] + 1, d);
            t[d] = (p[d] - posPrev(d)) / (posEnd(d) - posPrev(d));
        }
    }
    CoordinateDir minT = t[x];
    for (Axis d = 1; d < 3; d++) {
        if (t[d] < minT) {
            minT = t[d];
        }
    }
    cellNext = cellPrev;
    posNext = posPrev + (posEnd - posPrev) * minT;
    for (Axis d = 0; d < 3; d++) {
        if (t[d] == minT) {
            cellNext[d] = n[d];
            posNext[d] = p[d];
        }
    }
}





}
}