#include "MeshTools.h"

#include "Tools.h"
#include "Cleaner.h"
#include "GridTools.h"
#include "ElemGraph.h"

#include <sstream>

namespace meshlib {
namespace utils {
namespace meshTools {

Mesh duplicateCoordinatesUsedByDifferentGroups(const Mesh& mesh)
{
    Mesh res = mesh;

    std::map<CoordinateId, GroupId> usedIds;
    for (auto& g : res.groups) {
        GroupId gId = &g - &res.groups.front();
        std::map<CoordinateId, CoordinateId> remapedCoord;
        for (auto& e : g.elements) {
            for (auto& vId : e.vertices) {
                auto it = usedIds.find(vId);
                if (it != usedIds.end() && it->second != gId) {
                    if (remapedCoord.count(vId) == 0) {
                        Coordinate newCoord = res.coordinates[vId];
                        CoordinateId newVId = res.coordinates.size();
                        res.coordinates.push_back(newCoord);
                        remapedCoord.emplace(vId, newVId);
                        vId = newVId;
                        usedIds.emplace(newVId, gId);
                    }
                    else {
                        vId = remapedCoord[vId];
                    }
                }
                else {
                    usedIds.emplace(vId, gId);
                }
            }
        }
    }
    return res;
}

Grid getEnlargedGridIncludingAllElements(const Mesh& m)
{
    VecD bbMin, bbMax;
    std::tie(bbMin, bbMax) = getBoundingBox(m);

    Grid res = m.grid;
    for (std::size_t d = 0; d < 3; d++) {
        if (bbMin(d) < m.grid[d].front()) {
            res[d].insert(res[d].begin(), bbMin(d));
        }
        if (bbMax(d) > m.grid[d].back()) {
            res[d].push_back(bbMax(d));
        }
    }

    return res;
}

std::pair<VecD, VecD> getBoundingBox(const Mesh& m)
{
    VecD minBB(std::numeric_limits<double>::max());
    VecD maxBB(std::numeric_limits<double>::min());

    GridTools gT{ m.grid };
    Coordinates newPos{ m.coordinates };
    for (auto& pos : newPos) {
        Coordinate meshedPos{ gT.getPos(gT.getRelative(pos).round(1e6)) };
        if (GridTools::approx(meshedPos, pos, 1e-6)) {
            pos = meshedPos;
        }
    }

    for (auto const& g : m.groups) {
        for (auto const& e : g.elements) {
            for (auto const& vId : e.vertices) {
                const Coordinate& coord = newPos[vId];
                for (std::size_t d = 0; d < 3; d++) {

                    if (coord(d) < minBB(d)) {
                        minBB(d) = coord(d);
                    }
                    if (coord(d) > maxBB(d)) {
                        maxBB(d) = coord(d);
                    }
                }
            }
        }
    }

    return std::make_pair(minBB, maxBB);
}

void reduceGrid(Mesh& m, const Grid& nG)
{
    VecD offset = GridTools(m.grid).getOffsetWithGrid(nG).as<double>(); 
        
    m.grid = nG;

    Cleaner::removeElementsWithCondition(m, [&](const Element&e) {
        for (auto& vId : e.vertices) {
            Coordinate& c = m.coordinates[vId];
            for (std::size_t d = 0; d < 3; d++) {
                bool outOfLowerBound = (c(d) - offset(d)) < 0.0;
                bool outOfUpperBound = c(d) > (double)(m.grid[d].size()-1+offset(d));
                if (outOfLowerBound || outOfUpperBound) {
                    return true;
                }
            }
        }
        return false;
    });

	Cleaner::cleanCoords(m);
    for (auto& c : m.coordinates) {
        c -= offset;
    }

}

Mesh reduceGrid(const Mesh& m, const Grid& g)
{
    Mesh r{m};
    reduceGrid(r, g);
    return r;
}

Mesh setGrid(const Mesh& m, const Grid& g)
{
    Mesh aux;
    {
        Grid reducedGrid;
        for (const auto& x : { X, Y, Z }) {
            std::copy_if(
                m.grid[x].begin(), m.grid[x].end(),
                std::back_inserter(reducedGrid[x]),
                [&](const auto& v) {
                    return v >= g[x].front() && v <= g[x].back();
                }
            );
        }
        aux = reduceGrid(m, reducedGrid);
    }
    
    return Mesh{
        g,
        GridTools{ g }.absoluteToRelative(
            GridTools{ aux.grid }.relativeToAbsolute(
                aux.coordinates)),
        aux.groups
    };
}

void checkNoCellsAreCrossed(const Mesh& m)
{
    std::stringstream msg;
    GridTools gT(m.grid);
    bool crossesGrid = false;
    for (auto const& g : m.groups) {
        for (auto const& e : g.elements) {
            if (gT.elementCrossesGrid(e, m.coordinates)) {
                crossesGrid = true;
                msg << std::endl;
                msg << "Group: " << &g - &m.groups.front()
                    << ", Element: " << &e - &g.elements.front() << std::endl;
                msg << info(e, m) << std::endl;
            }
        }
    }
    if (crossesGrid) {
        msg << std::endl << "Invalid cell invariant: element spans more than one cell.";
        throw std::runtime_error(msg.str());
    }

}

void checkNoOverlaps(const Mesh& m)
{
    std::stringstream msg;
    double angle = 180;
    bool misoriented = false;
    for (const auto& g : m.groups) {
        ElemGraph eG(g.elements, m.coordinates);
        for (const auto& elPair : eG.findElementsWithWeight(180)) {
            misoriented = true;
            msg << std::endl;
            msg << "Group: " << &g - &m.groups.front()
                << ", Elements: " << elPair.first << " " << elPair.second << std::endl;
            msg << info(g.elements[elPair.first], m) << std::endl;
            msg << info(g.elements[elPair.second], m) << std::endl;
        }
    }
    if (misoriented) {
        msg << std::endl << "Invalid mesh invariant: adjacent elements overlap.";
        throw std::runtime_error(msg.str());
    }

}

void checkNoNullAreasExist(const Mesh& m)
{
    std::stringstream msg;
    bool nullAreas = false;
    for (const auto& g : m.groups) {
        for (auto const& e: g.elements) {
            if (Geometry::area(Geometry::asTriV(e, m.coordinates)) == 0.0) {
                nullAreas = true;
                msg << std::endl;
                msg << "Group: " << &g - &m.groups.front()
                    << ", Element: " << &e - &g.elements.front() << std::endl;
                msg << info(e, m) << std::endl;
            }
        }
    }
    if (nullAreas) {
        msg << std::endl << "Invalid mesh invariant: Null areas exist.";
        throw std::runtime_error(msg.str());
    }
}

void convertToAbsoluteCoordinates(Mesh& m)
{
    GridTools gT{ m.grid };

    std::transform(
        m.coordinates.begin(), m.coordinates.end(),
        m.coordinates.begin(),
        [&](const auto& v) { return gT.getPos(v); }
    );
}

void checkSlicedMeshInvariants(const Mesh& m)
{
    checkNoCellsAreCrossed(m);
    checkNoOverlaps(m);
    checkNoNullAreasExist(m);
}

Mesh buildMeshFilteringElements(
    const Mesh& in, std::function<bool(const Element&)> filter)
{
    Mesh r;
    r.grid = in.grid;
    r.coordinates = in.coordinates;
    r.groups.resize(in.groups.size());
    for (auto gId{ 0 }; gId < in.groups.size(); gId++) {
        const auto& inElems = in.groups[gId].elements;
        std::copy_if(
            inElems.begin(), inElems.end(), 
            std::back_inserter(r.groups[gId].elements), 
            filter);
    }
    return r;
}

void mergeGroup(Group& lG, const Group& rG, const CoordinateId& coordCount)
{
    auto& lElems{ lG.elements };
    lElems.reserve(lElems.size() + rG.elements.size());
    for (const auto& e : rG.elements) {
        lElems.push_back(e);
        for (std::size_t v = 0; v < lElems.back().vertices.size(); v++) {
            lElems.back().vertices[v] += coordCount;
        }
    }
}

void mergeMesh(Mesh& lMesh, const Mesh& iMesh)
{
    assert(lMesh.grid == iMesh.grid);
    assert(lMesh.groups.size() == iMesh.groups.size());

    auto coordCount{ lMesh.coordinates.size() };
    
    lMesh.coordinates.insert(lMesh.coordinates.end(),
        iMesh.coordinates.begin(), iMesh.coordinates.end());

    for (std::size_t g = 0; g < lMesh.groups.size(); g++) {
        mergeGroup(lMesh.groups[g], iMesh.groups[g], coordCount);
    }
}

void mergeMeshAsNewGroup(Mesh& lMesh, const Mesh& iMesh)
{
    assert(lMesh.grid == iMesh.grid);
    assert(iMesh.groups.size() == 1);

    auto coordCount{ lMesh.coordinates.size() };

    lMesh.coordinates.insert(lMesh.coordinates.end(),
        iMesh.coordinates.begin(), iMesh.coordinates.end());

    lMesh.groups.push_back(Group());
    mergeGroup(lMesh.groups.back(), iMesh.groups.front(), coordCount);
}

std::string info(const Element& e, const Mesh& m)
{
    std::stringstream r;
    r << "Element with vertices: " << std::endl;
    for (auto const& vId : e.vertices) {
        r << "[" << vId << "]"
            << " at: " << m.coordinates[vId].str() << std::endl;
    }
    return r.str();
}

}
}
}
