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

void getOverlappedDimensionZeroElementsAndIdenticalLines(const Group& group, std::set<ElementId>& overlappedElements);
void getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(const Group& group, const std::vector<Coordinate>& meshCoordinates, std::set<ElementId>& overlappedElements);

void RedundancyCleaner::removeOverlappedDimensionZeroElementsAndIdenticalLines(Mesh & mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];
        getOverlappedDimensionZeroElementsAndIdenticalLines(group, toRemove[g]);
    }

    removeElements(mesh, toRemove);
}

void RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(Mesh& mesh)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto& group = mesh.groups[g];
        getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(group, mesh.coordinates, toRemove[g]);
    }

    removeElements(mesh, toRemove);
}

void getOverlappedDimensionZeroElementsAndIdenticalLines(const Group& group, std::set<ElementId>& overlappedElements) {
    std::set<CoordinateId> usedCoordinates;
    std::vector<ElementId> nodesToCheck;

    for (std::size_t e = 0; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        if (element.isLine()) {
            usedCoordinates.insert(element.vertices[0]);
            usedCoordinates.insert(element.vertices[1]);
        }
        else if (element.isNode()) {
            nodesToCheck.push_back(e);
        }
    }

    for (auto e : nodesToCheck) {
        auto& node = group.elements[e];
        if (usedCoordinates.count(node.vertices[0]) == 0) {
            usedCoordinates.insert(node.vertices[0]);
        }
        else {
            overlappedElements.insert(e);
        }
    }
}

void getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(const Group& group, const std::vector<Coordinate> & meshCoordinates, std::set<ElementId>& overlappedElements)
{
    std::set<CoordinateIds> usedCoordinatesFromSurface;
    std::set<CoordinateIds> usedCoordinatePairsFromSurface;
    std::set<CoordinateId> usedCoordinates;
    std::vector<ElementId> linesToCheck;
    std::vector<ElementId> nodesToCheck;

    for (std::size_t e = 0; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        CoordinateIds vIds{ element.vertices };
        if (vIds.size() >= 2) {
            std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());
            for (std::size_t v = 0; v < vIds.size(); ++v) {
                usedCoordinates.insert(vIds[v]);
            }
        }
        if (element.isQuad() || element.isTriangle()) {
            if (usedCoordinatesFromSurface.count(vIds) == 0) {
                usedCoordinatesFromSurface.insert(vIds);
                for (std::size_t v = 0; v < vIds.size(); ++v) {
                    auto firstCoordinateId = vIds[v];
                    auto secondCoordinateId = vIds[(v + 1) % vIds.size()];

                    if (secondCoordinateId < firstCoordinateId) {
                        std::swap(firstCoordinateId, secondCoordinateId);
                    }
                    usedCoordinatePairsFromSurface.insert({ firstCoordinateId, secondCoordinateId });
                }
            }
            else {
                overlappedElements.insert(e);
            }
        }
        else if (element.isLine()) {
            linesToCheck.push_back(e);
        }
        else if (element.isNode()) {
            nodesToCheck.push_back(e);
        }
    }

    std::map<CoordinateIds, ElementId> usedCoordinatePairsFromLine;

    for (auto e : linesToCheck) {
        auto& line = group.elements[e];
        CoordinateIds vIds{ line.vertices };
        std::rotate(vIds.begin(), std::min_element(vIds.begin(), vIds.end()), vIds.end());

        if (usedCoordinatePairsFromSurface.count(vIds)) {
            overlappedElements.insert(e);
        }
        else if (usedCoordinatePairsFromLine.count(vIds) == 0) {
            usedCoordinatePairsFromLine.emplace(vIds, e);
        }
        else {
            auto& originalLine = group.elements[usedCoordinatePairsFromLine[vIds]];
            RelativeDir direction = 0;
            RelativeDir originalDirection = 0;
            for (auto axis = X; axis <= Z; ++axis) {
                direction += meshCoordinates[line.vertices[1]][axis] - meshCoordinates[line.vertices[0]][axis];
                originalDirection += meshCoordinates[originalLine.vertices[1]][axis] - meshCoordinates[originalLine.vertices[0]][axis];
            }

            if (direction > originalDirection) {
                overlappedElements.insert(usedCoordinatePairsFromLine[vIds]);
                usedCoordinatePairsFromLine[vIds] = e;
            }
            else {
                overlappedElements.insert(e);
            }
        }
    }

    for (auto e : nodesToCheck) {
        auto& node = group.elements[e];
        if (usedCoordinates.count(node.vertices[0]) == 0) {
            usedCoordinates.insert(node.vertices[0]);
        }
        else {
            overlappedElements.insert(e);
        }
    }
}


void RedundancyCleaner::removeOverlappedElementsByDimension(Mesh& mesh, const std::vector<Element::Type>& highestDimensions)
{
    std::vector<std::set<ElementId>> toRemove(mesh.groups.size());

    for (std::size_t g = 0; g < mesh.groups.size(); ++g) {
        auto & group = mesh.groups[g];

        switch (highestDimensions[g]) {
            case Element::Type::Surface:
                getOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(group, mesh.coordinates, toRemove[g]);
                break;
            case Element::Type::Line:
                getOverlappedDimensionZeroElementsAndIdenticalLines(group, toRemove[g]);
            default:
                break;
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

void RedundancyCleaner::fuseCoords(Mesh& mesh) 
{
    std::map<Coordinate, IdSet> posIds;
    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            const Element& elem = mesh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId id = elem.vertices[i];
                Coordinate pos = mesh.coordinates[id];
                posIds[pos].insert(id);
            }
        }
    }

    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            Element& elem = mesh.groups[g].elements[e];
            for (std::size_t i = 0; i < elem.vertices.size(); i++) {
                CoordinateId oldMeshedId = elem.vertices[i];
                CoordinateId newMeshedId = *posIds[mesh.coordinates[oldMeshedId]].begin();
                std::replace(elem.vertices.begin(), elem.vertices.end(), oldMeshedId, newMeshedId);
            }
        }
    }
}

void RedundancyCleaner::removeDegenerateElements(Mesh& mesh){
    removeElementsWithCondition(mesh, [&](const Element& e) {
        return IdSet(e.vertices.begin(), e.vertices.end()).size() != e.vertices.size();
    });
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