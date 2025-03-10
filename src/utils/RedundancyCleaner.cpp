#include "RedundancyCleaner.h"

#include "Geometry.h"
#include "GridTools.h"

#include "MeshTools.h"

#include <map>
#include <set>
#include <algorithm>
#include <unordered_set> 

namespace meshlib {
namespace utils {

void RedundancyCleaner::removeRepeatedElementsIgnoringOrientation(Mesh& m)
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

void RedundancyCleaner::removeRepeatedElements(Mesh& m)
{
    std::vector<std::set<ElementId>> toRemove(m.groups.size());
    for (const auto& g : m.groups) {
        auto gId{&g - &m.groups.front()};
        std::map<CoordinateIds, ElementId> vToE;
        for (const auto& e : g.elements) {
            auto eId{ &e - &g.elements.front() };
            CoordinateIds vIds{ e.vertices };
            if (vIds.size() > 2) {
                std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
            }
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

void RedundancyCleaner::removeOverlappedDimensionZeroElementsAndIdenticalLines(Mesh & mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];
        std::set<CoordinateId> usedCoordinates;
        std::vector<ElementId> nodesToCheck;

        for (std::size_t e = 0; e < group.elements.size(); ++e){
            auto& element = group.elements[e];
            if(element.isLine()){
                usedCoordinates.insert(element.vertices[0]);
                usedCoordinates.insert(element.vertices[1]);
            }
            else if (element.isNode()){
                nodesToCheck.push_back(e);
            }
        }

        for(auto e : nodesToCheck){
            auto & node = group.elements[e];
            if (usedCoordinates.count(node.vertices[0]) == 0){
                usedCoordinates.insert(node.vertices[0]);
            }
            else{
                toRemove[g].insert(e);
            }
        }
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(Mesh & mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];

        std::set<CoordinateIds> usedCoordinatesFromSurface;
        std::set<CoordinateIds> usedCoordinatePairsFromSurface;
        std::set<CoordinateId> usedCoordinates;
        std::vector<ElementId> linesToCheck;
        std::vector<ElementId> nodesToCheck;

        for (std::size_t e = 0; e < group.elements.size(); ++e){
            auto& element = group.elements[e];
            CoordinateIds vIds{ element.vertices };
            if(vIds.size() >= 2){
                std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
                for (std::size_t v = 0; v < vIds.size(); ++v){
                    usedCoordinates.insert(vIds[v]);
                }
            }
            if(element.isQuad() || element.isTriangle()){
                if (usedCoordinatesFromSurface.count(vIds) == 0){
                    usedCoordinatesFromSurface.insert(vIds);
                    for (std::size_t v = 0; v < vIds.size(); ++v){
                        auto firstCoordinateId = vIds[v];
                        auto secondCoordinateId = vIds[(v + 1) % vIds.size()];

                        if (secondCoordinateId < firstCoordinateId){
                            std::swap(firstCoordinateId, secondCoordinateId);
                        }
                        usedCoordinatePairsFromSurface.insert({firstCoordinateId, secondCoordinateId});
                    }
                }
                else{
                    toRemove[g].insert(e);
                }
            }
            else if (element.isLine()){
                linesToCheck.push_back(e);
            }
            else if (element.isNode()){
                nodesToCheck.push_back(e);
            }
        }
        
        std::map<CoordinateIds, ElementId> usedCoordinatePairsFromLine;

        for (auto e : linesToCheck){
            auto & line = group.elements[e];
            CoordinateIds vIds{ line.vertices };
            std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());

            if (usedCoordinatePairsFromSurface.count(vIds)){
                toRemove[g].insert(e);
            }
            else if(usedCoordinatePairsFromLine.count(vIds) == 0){
                    usedCoordinatePairsFromLine.emplace(vIds, e);
            }
            else{
                auto & originalLine = group.elements[usedCoordinatePairsFromLine[vIds]];
                RelativeDir direction = 0;
                RelativeDir originalDirection = 0;
                for (auto axis = X; axis <= Z; ++axis){
                    direction += mesh.coordinates[line.vertices[1]][axis] - mesh.coordinates[line.vertices[0]][axis];
                    originalDirection += mesh.coordinates[originalLine.vertices[1]][axis] - mesh.coordinates[originalLine.vertices[0]][axis];
                }

                if (direction > originalDirection){
                    toRemove[g].insert(usedCoordinatePairsFromLine[vIds]);
                    usedCoordinatePairsFromLine[vIds] = e;
                }
                else{
                    toRemove[g].insert(e);
                }
            }
        }

        for(auto e : nodesToCheck){
            auto & node = group.elements[e];
            if (usedCoordinates.count(node.vertices[0]) == 0){
                usedCoordinates.insert(node.vertices[0]);
            }
            else{
                toRemove[g].insert(e);
            }
        }
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeElementsWithCondition(Mesh& m, std::function<bool(const Element&)> cnd)
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

Elements RedundancyCleaner::findDegenerateElements_(
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

void RedundancyCleaner::collapseCoordsInLineDegenerateTriangles(Mesh& m, const double& areaThreshold) 
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
            if (e.isNode() || e.isLine()) {
                continue;
            }

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

void RedundancyCleaner::fuseCoords(Mesh& mesh, bool cleanDegeneracy) 
{
    fuseCoords_(mesh);
    if(cleanDegeneracy){
        removeElementsWithCondition(mesh, [&](const Element& e) {
            return IdSet(e.vertices.begin(), e.vertices.end()).size() != e.vertices.size();
        });
    }
    
}

void RedundancyCleaner::cleanCoords(Mesh& output) 
{
    const std::size_t& numStrCoords = output.coordinates.size();

    IdSet coordsUsed;
    
    for (auto const& g: output.groups) {
        for (auto const& e: g.elements) {
                coordsUsed.insert(e.vertices.begin(), e.vertices.end());
        }
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
    
}

void RedundancyCleaner::fuseCoords_(Mesh& msh) 
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

void RedundancyCleaner::removeElements(Mesh& mesh, const std::vector<IdSet>& toRemove) 
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