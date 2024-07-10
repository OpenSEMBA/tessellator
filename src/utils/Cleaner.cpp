#include "Cleaner.h"

#include "Geometry.h"
#include "GridTools.h"

#include "MeshTools.h"

#include <map>
#include <set>
#include <algorithm>
#include <unordered_set> 

namespace meshlib {
namespace utils {

void Cleaner::clean(Mesh& output, Map& map) 
{
    cleanElems_ (output, map);
    cleanCoords_(output, map);
}

void Cleaner::cleanCoords(Mesh& output) 
{
    Map map;
    cleanCoords_(output, map);
}

void Cleaner::removeRepeatedElementsIgnoringOrientation(Mesh& m)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (const auto& g : m.groups) {
        auto gId{ &g - &m.groups.front() };
        std::map<IdSet, ElementId> vToE;
        for (const auto& e : g.elements) {
            auto eId{ &e - &g.elements.front() };
            IdSet vIds{ e.vertices.begin(), e.vertices.end() };
            if (vToE.count(vIds) == 0) {
                vToE.emplace(vIds, eId);
            }
            else {
                toRemove[gId].insert(eId);
            }
        }
    }

    removeElements(m, toRemove);
}

void Cleaner::removeRepeatedElements(Mesh& m)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (const auto& g : m.groups) {
        auto gId{&g - &m.groups.front()};
        std::map<CoordinateIds, ElementId> vToE;
        for (const auto& e : g.elements) {
            auto eId{ &e - &g.elements.front() };
            CoordinateIds vIds{ e.vertices };
            std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
            if (vToE.count(vIds) == 0) {
                vToE.emplace(vIds, eId);
            }
            else {
                toRemove[gId].insert(eId);
            }
        }
    }

    removeElements(m, toRemove);
}

void Cleaner::removeElementsWithCondition(Mesh& m, std::function<bool(const Element&)> cnd)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (auto const& g : m.groups) {
        const GroupId gId = &g - &m.groups.front();
        for (auto const& e : g.elements) {
            const ElementId eId = &e - &g.elements.front();
            if (cnd(e)) {
                toRemove[gId].insert(eId);
            }
        }
    }
    removeElements(m, toRemove);
}

Elements Cleaner::findDegenerateElements_(
    const Group& g,
    const Coordinates& coords)
{
    Elements res;
    for (const auto e : g.elements) {
        if (!e.isTriangle()) {
            continue;
        }
        if (Geometry::isDegenerate(Geometry::asTriV(e, coords))) {
            res.push_back(e);
        }
    }
    return res;
}

void Cleaner::collapseCoordsInLineDegenerateTriangles(Mesh& m, const double& areaThreshold) 
{
    const std::size_t MAX_NUMBER_OF_ITERATION = 1000;
    bool degeneratedTrianglesFound = true;
    for (std::size_t iter = 0; 
        iter < MAX_NUMBER_OF_ITERATION && degeneratedTrianglesFound; 
        ++iter) 
    {
        degeneratedTrianglesFound = false;
        for (auto& g : m.groups) {
            for (auto& e : g.elements) {
                if (!e.isTriangle() ||
                    !Geometry::isDegenerate(Geometry::asTriV(e, m.coordinates), areaThreshold)) {
                    continue;
                }
                degeneratedTrianglesFound = true;
                Coordinates& coords = m.coordinates;
                const std::vector<CoordinateId>& v = e.vertices;
                std::pair<std::size_t, CoordinateId> replace;

                std::array<double, 3> sumOfDistances{ 0,0,0 };
                for (std::size_t d : {0, 1, 2}) {
                    for (std::size_t dd : {1, 2}) {
                        sumOfDistances[d] += (coords[v[d]] - coords[v[(d + dd) % 3]]).norm();
                    }
                }
                auto minPos = std::min_element(sumOfDistances.begin(), sumOfDistances.end());
                auto midId = std::distance(sumOfDistances.begin(), minPos);

                const auto& cMid = coords[v[midId]];
                const auto& cExt1 = coords[v[(midId + 1) % 3]];
                const auto& cExt2 = coords[v[(midId + 2) % 3]];

                if ((cMid - cExt1).norm() < (cMid - cExt2).norm()) {
                    coords[e.vertices[midId]] = coords[e.vertices[(midId + 1) % 3]];
                }
                else {
                    coords[e.vertices[midId]] = coords[e.vertices[(midId + 2) % 3]];
                }
            }
        }

        fuseCoords(m);
        cleanCoords(m);
    }
     
    std::stringstream msg;
    bool breaksPostCondition = false;
    for (auto const& g : m.groups) {
        for (auto const& e : g.elements) {
            double area = Geometry::area(Geometry::asTriV(e, m.coordinates));
            if (e.isTriangle() && area < areaThreshold) {
                breaksPostCondition = true;
                msg << std::endl;
                msg << "Group: " << &g - &m.groups.front()
                    << ", Element: " << &e - &g.elements.front() << std::endl;
                msg << meshTools::info(e, m) << std::endl;
            }
        }
    }
    if (breaksPostCondition) {
        msg << std::endl << "Triangles with area above threshold exist after collapsing.";
        throw std::runtime_error(msg.str());
    }
}

void Cleaner::fuseCoords(Mesh& mesh) 
{
    fuseCoords_(mesh);
    removeElementsWithCondition(mesh, [&](const Element& e) {
        return IdSet(e.vertices.begin(), e.vertices.end()).size() != e.vertices.size();
    });
}


void Cleaner::cleanElems_(Mesh& output, Map& map) 
{
    std::size_t numUnstrElems = 0;
    std::size_t numStrElems = 0;
    for (GroupId g = 0; g < map.groups.size(); g++) {
        numUnstrElems += map.groups[g].elements.size();
        numStrElems   += output.groups[g].elements.size();
    }
    
    for (GroupId g = 0; g < map.groups.size(); g++) {
        std::vector<bool> elemsUsed(output.groups[g].elements.size(), false);
        for (ElementId e = 0; e < map.groups[g].elements.size(); e++) {
            for (ElementId
                 mE = 0; mE < map.groups[g].elements[e].size(); mE++) {
                elemsUsed[map.groups[g].elements[e][mE]] = true;
            }
        }
        std::map<ElementId, ElementId> elemsMap;
        std::vector<Element> aux = output.groups[g].elements;
        output.groups[g].elements.clear();
        for (ElementId e = 0; e < aux.size(); e++) {
            if (elemsUsed[e]) {
                elemsMap[e] = output.groups[g].elements.size();
                output.groups[g].elements.push_back(aux[e]);
            }
            
        }
        for (ElementId e = 0; e < map.groups[g].elements.size(); e++) {
            Map::Element aux = map.groups[g].elements[e];
            map.groups[g].elements[e].clear();
            for (ElementId mE = 0; mE < aux.size(); mE++) {
                if (elemsMap.count(aux[mE]) != 0) {
                    map.groups[g].elements[e].push_back(elemsMap[aux[mE]]);
                }
            }
            
        }
    }
    
}

void Cleaner::cleanCoords_(Mesh& output, Map& map) 
{
    const std::size_t& numUnstrCoords = map.coordinates.size();
    const std::size_t& numStrCoords = output.coordinates.size();

    IdSet coordsUsed;
    
    for (auto const& g: output.groups) {
        for (auto const& e: g.elements) {
                coordsUsed.insert(e.vertices.begin(), e.vertices.end());
        }
    }
    for (auto const strCoords: map.coordinates) {
            coordsUsed.insert(strCoords.begin(), strCoords.end());
    }

    std::map<CoordinateId, CoordinateId> remap;
    std::vector<Coordinate> aux = output.coordinates;
    output.coordinates.clear();
    for (CoordinateId c = 0; c < aux.size(); c++) {
        if (coordsUsed.count(c) != 0) {
            remap[c] = output.coordinates.size();
            output.coordinates.push_back(aux[c]);
        }
        
    }
    for (GroupId g = 0; g < output.groups.size(); g++) {
        for (ElementId e = 0; e < output.groups[g].elements.size(); e++) {
            Element& elem = output.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                elem.vertices[i] = remap[elem.vertices[i]];
            }
            
        }
    }

    for (CoordinateId c = 0; c < map.coordinates.size(); c++) {
        for (std::size_t i = 0; i < map.coordinates[c].size(); i++) {
            map.coordinates[c][i] = remap[map.coordinates[c][i]];
        }       
    }
    
}

void Cleaner::fuseCoords_(Mesh& msh) 
{
    std::map<Coordinate, IdSet> posIds;
    for (GroupId g = 0; g < msh.groups.size(); g++) {
        for (ElementId e = 0; e < msh.groups[g].elements.size(); e++) {
            const Element& elem = msh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId id = elem.vertices[i];
                Coordinate pos = msh.coordinates[id];
                posIds[pos].insert(id);
            }
        }
    }

    for (GroupId g = 0; g < msh.groups.size(); g++) {
        for (ElementId e = 0; e < msh.groups[g].elements.size(); e++) {
            Element& elem = msh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId oldMeshedId = elem.vertices[i];
                CoordinateId newMeshedId = *posIds[msh.coordinates[oldMeshedId]].begin();
                std::replace(elem.vertices.begin(), elem.vertices.end(), oldMeshedId, newMeshedId);
            }
        }
    }
}

void Cleaner::removeElements(Mesh& mesh, const std::vector<IdSet>& toRemove) 
{
    for (GroupId gId = 0; gId < mesh.groups.size(); gId++) {
        Elements& elems = mesh.groups[gId].elements;
        Elements newElems;
        newElems.reserve(elems.size() - toRemove[gId].size());
        auto it = toRemove[gId].begin();
        for (std::size_t i = 0; i < elems.size(); i++) {
            if (it == toRemove[gId].end() || i != *it) {
                newElems.push_back(elems[i]);
            }
            else {
                ++it;
            }
        }

        elems = newElems;       
    }
}

}
}