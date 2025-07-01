#include "Staircaser.h"

#include "utils/RedundancyCleaner.h"

#include <iostream>

namespace meshlib {
namespace core {
using namespace utils;

Staircaser::Staircaser(const Mesh& inputMesh) : GridTools(inputMesh.grid)
{
    inputMesh_ = inputMesh;

    mesh_.grid = inputMesh.grid;

    mesh_.coordinates.reserve(inputMesh.coordinates.size() * 2);

    mesh_.groups.resize(inputMesh.groups.size());

}

Mesh Staircaser::getMesh()
{
    for (std::size_t g = 0; g < mesh_.groups.size(); ++g) {

        auto& inputGroup = inputMesh_.groups[g];
        auto& meshGroup = mesh_.groups[g];
        meshGroup.elements.reserve(inputGroup.elements.size() * 2);

        for (auto & element : inputGroup.elements) {
            if (element.isNode()) {
                this->processNodeAndAddToGroup(element, inputMesh_.coordinates, mesh_.coordinates, meshGroup);
            }
            else if (element.isLine()) {
                this->processLineAndAddToGroup(element, inputMesh_.coordinates, mesh_.coordinates, meshGroup);
            }
            else if (element.isTriangle()) {
                this->processTriangleAndAddToGroup(element, inputMesh_.coordinates, meshGroup);
            }
        }
    }

    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::removeDegenerateElements(mesh_);
    RedundancyCleaner::cleanCoords(mesh_);

    return mesh_;
}

CoordinateMap buildCoordinateMap(const Coordinates& cs) 
{
    CoordinateMap res;
    for (std::size_t c = 0; c < cs.size(); ++c) {
        res[cs[c]] = c;       
    }
    return res;
}

IdSet findCommonNeighborsVertices(const Mesh& mesh, const std::pair<CoordinateId, CoordinateId>& edge)
{
    IdSet commonNeighborsVertices;

    const auto& vertex1 = edge.first;
    const auto& vertex2 = edge.second;

    GridTools gridTools;
    auto cellElemMap = gridTools.buildCellElemMap(mesh.groups[0].elements, mesh.coordinates);
    CoordinateMap coordinateMap = buildCoordinateMap(mesh.coordinates);

    auto cellsCoord1 = gridTools.getTouchingCells(mesh.coordinates[vertex1]);
    auto cellsCoord2 = gridTools.getTouchingCells(mesh.coordinates[vertex2]);

    IdSet neighborsOfVertex1;
    IdSet neighborsOfVertex2;

    for (const auto& c : cellsCoord1) {
        for (const auto& e : cellElemMap.at(c)) {
            const auto& elementVertices = e->vertices;
            auto it = std::find(elementVertices.begin(), elementVertices.end(), vertex1);

            if (it != elementVertices.end()) {
                auto pos = std::distance(elementVertices.begin(), it);
                auto n = elementVertices.size();

                neighborsOfVertex1.insert(elementVertices[(pos + n - 1) % n]); 
                neighborsOfVertex1.insert(elementVertices[(pos + 1) % n]); 
            }        
        }
    }

    for (const auto& c : cellsCoord2) {
        for (const auto& e : cellElemMap.at(c)) {
            const auto& elementVertices = e->vertices;
            auto it = std::find(elementVertices.begin(), elementVertices.end(), vertex2);

            if (it != elementVertices.end()) {
                auto pos = std::distance(elementVertices.begin(), it);
                auto n = elementVertices.size();

                neighborsOfVertex2.insert(elementVertices[(pos + n - 1) % n]); 
                neighborsOfVertex2.insert(elementVertices[(pos + 1) % n]); 
            }
        }
    }

    for (const auto& v : neighborsOfVertex1) {
        if (neighborsOfVertex2.count(v)) {
            commonNeighborsVertices.insert(v);
        }
    }

    return commonNeighborsVertices;
}

Elements findTrianglesWithEdge(const Mesh& mesh, const std::pair<CoordinateId, CoordinateId>& edge) {
    Elements foundTriangles;
    const auto& v1 = edge.first;
    const auto& v2 = edge.second;

    for (const auto& element : mesh.groups[0].elements) {
        if (element.vertices.size() == 3) {
            auto it1 = std::find(element.vertices.begin(), element.vertices.end(), v1);
            auto it2 = std::find(element.vertices.begin(), element.vertices.end(), v2);

            if (it1 != element.vertices.end() && it2 != element.vertices.end()) {
                foundTriangles.push_back(element);
            }
        }
    }
    return foundTriangles;
}

Mesh Staircaser::getSelectiveMesh(const std::set<Cell>& cellsToStructure, GapsFillingType type)
{
    fillerType_ = type;

    RelativePairSet boundaryCoordinatePairs;
    for (std::size_t g = 0; g < mesh_.groups.size(); ++g) {

        auto& inputGroup = inputMesh_.groups[g];
        auto& meshGroup = mesh_.groups[g];
        meshGroup.elements.reserve(inputGroup.elements.size() * 2);

        auto cellElemMap = buildCellElemMap(inputGroup.elements, inputMesh_.coordinates);
        
        for (const auto& c : cellsToStructure) {
            if (!cellElemMap.count(c)) {
                continue;
            }
            for (const auto e:  cellElemMap.at(c)) {
                if (e->isLine()) {  
                    this->processLineAndAddToGroup(*e, inputMesh_.coordinates, mesh_.coordinates, meshGroup);
                }
                else if (e->isTriangle()) {
                    this->processTriangleAndAddToGroup(*e, inputMesh_.coordinates, meshGroup);
                }
            }
        } 

        CoordinateMap coordinateMap = buildCoordinateMap(mesh_.coordinates);

        meshGroup.elements.reserve(meshGroup.elements.size() + inputGroup.elements.size());
        for (const auto& [cell, elements] : cellElemMap) {
            if (cellsToStructure.count(cell) ) {
                continue;
            }
            for (const auto e : elements) {
                Element newElement;
                newElement.type = e->type;
                newElement.vertices.reserve(e->vertices.size());

                Relatives boundaryCoordinates;

                for (const auto& vertexIndex : e->vertices) {
                    const auto& vertexCoord = inputMesh_.coordinates[vertexIndex];
                    
                    bool isOnCellBoundary = false;
                    auto touchingCells = GridTools::getTouchingCells(vertexCoord);
                    
                    for (const auto& touchingCell : touchingCells) {
                        if (cellsToStructure.count(touchingCell)) {
                            isOnCellBoundary = true;
                            break;
                        }
                    }
                    
                    CoordinateId newIndex;
                    if (isOnCellBoundary) {
                        auto it = coordinateMap.find(toRelative(calculateStaircasedCell(vertexCoord)));
                        newIndex = it->second;
                        boundaryCoordinates.push_back(toRelative(calculateStaircasedCell(vertexCoord)));
                    } else {
                        auto it = coordinateMap.find(vertexCoord);
                        if (it == coordinateMap.end()) {
                            mesh_.coordinates.push_back(vertexCoord);
                            newIndex = int (mesh_.coordinates.size() - 1);
                            coordinateMap.emplace(vertexCoord, newIndex);
                        } else {
                            newIndex = it->second;
                        }
                    }
                    newElement.vertices.push_back(newIndex);
                }               

                bool isAllCoordinatesOnCellBoundary = true;

                for (const auto& vertex : newElement.vertices) {
                    const auto& vertexCoord = mesh_.coordinates[vertex];
                    auto touchingCells = GridTools::getTouchingCells(vertexCoord);
                    
                    bool vertexOnBoundary = false;
                    for (const auto& touchingCell : touchingCells) {
                        if (cellsToStructure.count(touchingCell)) {
                            vertexOnBoundary = true;
                            break;
                        } 
                    }

                    if(!vertexOnBoundary) {
                        isAllCoordinatesOnCellBoundary = false;
                        break;
                    }
                }

                if (!isAllCoordinatesOnCellBoundary) {
                    meshGroup.elements.push_back(newElement);
    
                    for (size_t i = 0; i < boundaryCoordinates.size(); ++i) {
                        for (size_t j = i + 1; j < boundaryCoordinates.size(); ++j) {
                            boundaryCoordinatePairs.emplace(boundaryCoordinates[i], boundaryCoordinates[j]);
                        }
                    }
                }
            }
        }
    }

    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::removeDegenerateElements(mesh_);
    RedundancyCleaner::cleanCoords(mesh_);

    for (auto it = boundaryCoordinatePairs.begin(); it != boundaryCoordinatePairs.end();) {
        const auto& [coord1, coord2] = *it;
        
        bool alignedInXY = (coord1[2] == coord2[2]) && (coord1[0] == coord2[0] || coord1[1] == coord2[1]);
        bool alignedInXZ = (coord1[1] == coord2[1]) && (coord1[0] == coord2[0] || coord1[2] == coord2[2]);
        bool alignedInYZ = (coord1[0] == coord2[0]) && (coord1[1] == coord2[1] || coord1[2] == coord2[2]);


        if (alignedInXY || alignedInXZ || alignedInYZ) {
            it = boundaryCoordinatePairs.erase(it);
        } else {
            ++it;
        }
    }

    fillGaps(boundaryCoordinatePairs);   

    return mesh_;
}

void Staircaser::fillGaps(const RelativePairSet boundaryCoordinatePairs) {
    std::set<std::vector<ElementId>> uniqueElementsByVertices;
    for (const auto& element : mesh_.groups[0].elements) {
        uniqueElementsByVertices.insert(element.vertices);
    }

    CoordinateMap coordinateMap = buildCoordinateMap(mesh_.coordinates);
    for (const auto& [coord1, coord2] : boundaryCoordinatePairs) {
        auto v1 = coordinateMap.at(coord1);
        auto v2 = coordinateMap.at(coord2);
        std::pair<CoordinateId, CoordinateId> edge = std::make_pair(v1, v2);

        auto commonNeighbors = findCommonNeighborsVertices(mesh_, edge);

        auto triangles = findTrianglesWithEdge(mesh_, edge);
        bool correctOrientation;
        ElementId thirdVertex;

        if (!triangles.empty()) {
            for (const auto& elem : triangles) {
                thirdVertex = -1;

                const auto& verts = elem.vertices; 
                auto it1 = std::find(verts.begin(), verts.end(), v1);
                auto it2 = std::find(verts.begin(), verts.end(), v2);

                int idx1 = std::distance(verts.begin(), it1);
                int idx2 = std::distance(verts.begin(), it2);

                int nextIdx1 = (idx1 + 1) % 3;
                int prevIdx1 = (idx1 + 2) % 3;

                if (idx2 == nextIdx1) {
                    thirdVertex = verts[(idx2 + 1) % 3];
                    correctOrientation = true;
                } else if (idx2 == prevIdx1) {
                    thirdVertex = verts[(idx1 + 1) % 3];
                    correctOrientation = false;
                }
            }
        }

        if (fillerType_ == GapsFillingType::Insert) {
            for (const auto& neighborVertex : commonNeighbors) {
                Element triangle;
                triangle.type = Element::Type::Surface;
                triangle.vertices = { v1, v2, neighborVertex };

                if (!correctOrientation) {
                    std::swap(triangle.vertices[1], triangle.vertices[2]);
                }

                auto minIt = std::min_element(triangle.vertices.begin(), triangle.vertices.end());
                std::rotate(triangle.vertices.begin(), minIt, triangle.vertices.end());

                bool elementAlreadyExists = false;

                for (const auto& existing : uniqueElementsByVertices) {
                    auto existingVerts = existing;
                    auto minItExisting = std::min_element(existingVerts.begin(), existingVerts.end());
                    std::rotate(existingVerts.begin(), minItExisting, existingVerts.end());

                    if (existingVerts == triangle.vertices) {
                        elementAlreadyExists = true;
                        break;
                    }
                }

                if (!elementAlreadyExists) {
                    std::swap(triangle.vertices[1], triangle.vertices[2]);
                    mesh_.groups[0].elements.push_back(triangle);
                    uniqueElementsByVertices.insert(triangle.vertices);
                }
            }
        } else if (fillerType_ == GapsFillingType::Split) {
            for (const auto& neighborVertex : commonNeighbors) {
                if (neighborVertex != thirdVertex) {
                    Element triangleToRemove;
                    triangleToRemove.type = Element::Type::Surface;
                    triangleToRemove.vertices = { v1, v2, thirdVertex };

                    Element triangle1;
                    triangle1.type = Element::Type::Surface;
                    triangle1.vertices = { v1, neighborVertex, thirdVertex };

                    Element triangle2;
                    triangle2.type = Element::Type::Surface;
                    triangle2.vertices = { neighborVertex, v2, thirdVertex };

                    if (!correctOrientation) {
                        std::swap(triangleToRemove.vertices[1], triangleToRemove.vertices[2]);
                        std::swap(triangle1.vertices[1], triangle1.vertices[2]);
                        std::swap(triangle2.vertices[1], triangle2.vertices[2]);
                    }

                    auto minIt = std::min_element(triangleToRemove.vertices.begin(), triangleToRemove.vertices.end());
                    std::rotate(triangleToRemove.vertices.begin(), minIt, triangleToRemove.vertices.end());

                    auto minIt1 = std::min_element(triangle1.vertices.begin(), triangle1.vertices.end());
                    std::rotate(triangle1.vertices.begin(), minIt1, triangle1.vertices.end());

                    auto minIt2 = std::min_element(triangle2.vertices.begin(), triangle2.vertices.end());
                    std::rotate(triangle2.vertices.begin(), minIt2, triangle2.vertices.end());

                    mesh_.groups[0].elements.push_back(triangle1);
                    mesh_.groups[0].elements.push_back(triangle2);

                    std::vector<IdSet> toRemove(mesh_.groups.size());
                    for (GroupId g = 0; g < mesh_.groups.size(); ++g) {
                        const auto& elements = mesh_.groups[g].elements;
                        for (std::size_t i = 0; i < elements.size(); ++i) {
                            if (elements[i].type != Element::Type::Surface) {
                                continue;
                            }

                            auto verts = elements[i].vertices;
                            auto minIt = std::min_element(verts.begin(), verts.end());
                            std::rotate(verts.begin(), minIt, verts.end());

                            if (verts == triangleToRemove.vertices) {
                                toRemove[g].insert(i);
                            }
                        }
                    }

                    RedundancyCleaner::removeElements(mesh_, toRemove);
                }
            }
        } else {
            break;
        }
    }

    RedundancyCleaner::removeDegenerateElements(mesh_);
}

void Staircaser::processTriangleAndAddToGroup(const Element& triangle, const Relatives& originalRelatives, Group& group){
    Group edges;

    edges.elements = {
        Element({triangle.vertices[0], triangle.vertices[1]}, Element::Type::Line),
        Element({triangle.vertices[1], triangle.vertices[2]}, Element::Type::Line),
        Element({triangle.vertices[2], triangle.vertices[0]}, Element::Type::Line),
    };

    Mesh auxiliarMesh;
    auxiliarMesh.grid = this->mesh_.grid;
    auxiliarMesh.groups = { Group() };
    Group& processedEdges = auxiliarMesh.groups[0];
    auxiliarMesh.groups[0].elements.reserve(9);
    int pureDiagonalIndex = -1;

    for (std::size_t index = 0; index < edges.elements.size(); ++index) {
        auto& edge = edges.elements[index];
        if (isPureDiagonal(edge, originalRelatives)) {
            pureDiagonalIndex = int(index);
        }
        else {
            this->processLineAndAddToGroup(edge, originalRelatives, auxiliarMesh.coordinates, processedEdges);
        }
    }

    RedundancyCleaner::fuseCoords(auxiliarMesh);
    RedundancyCleaner::removeDegenerateElements(auxiliarMesh);
    RedundancyCleaner::cleanCoords(auxiliarMesh);

    std::map<Surfel, IdSet> idSetByCellSurface;
    std::map<Surfel, RelativeIds> relativeIdsByCellSurface;

    calculateRelativeIdSetByCellSurface(auxiliarMesh.coordinates, idSetByCellSurface);

    filterSurfacesFromRelativeIds(
        triangle.vertices,
        pureDiagonalIndex,
        originalRelatives,
        idSetByCellSurface,
        auxiliarMesh.coordinates,
        relativeIdsByCellSurface);

    if(auxiliarMesh.coordinates.size() == 6 && relativeIdsByCellSurface.size() == 0){
        relativeIdsByCellSurface.clear();
        idSetByCellSurface.clear();

        this->addNewRelativeToGroupUsingBarycentre(triangle.vertices, originalRelatives, auxiliarMesh.coordinates, processedEdges);
        
        calculateRelativeIdSetByCellSurface(auxiliarMesh.coordinates, idSetByCellSurface);

        filterSurfacesFromRelativeIds(
            triangle.vertices,
            pureDiagonalIndex,
            originalRelatives,
            idSetByCellSurface,
            auxiliarMesh.coordinates,
            relativeIdsByCellSurface);
        }

    RelativeId newRelativeId = this->mesh_.coordinates.size();

    mesh_.coordinates.insert(mesh_.coordinates.end(), auxiliarMesh.coordinates.begin(), auxiliarMesh.coordinates.end());

    std::vector<bool> surfacePresenceList = {false, false};
    std::vector<Surfel> cellSurfacePlanes;
    std::vector<RelativeIds*> cellSurfaceIdsList;

    if (relativeIdsByCellSurface.size() >= 2) {
        surfacePresenceList.clear();
        surfacePresenceList.reserve(3);
        cellSurfacePlanes.reserve(3);
        cellSurfaceIdsList.reserve(3);

        for(auto it = relativeIdsByCellSurface.begin(); it != relativeIdsByCellSurface.end(); ++it){
            surfacePresenceList.push_back(true);
            cellSurfacePlanes.push_back(it->first);
            cellSurfaceIdsList.push_back(&it->second);
        }

        for(std::size_t ceiling = cellSurfacePlanes.size() - 1; ceiling > 0; --ceiling){
            for(std::size_t planeIndex = 0; planeIndex < ceiling; ++planeIndex){
                auto& firstCellSurfacePlane = cellSurfacePlanes[planeIndex];
                auto& secondCellSurfacePlane = cellSurfacePlanes[planeIndex+1];
                auto& firstCellSurfaceIds = cellSurfaceIdsList[planeIndex];
                auto& secondCellSurfaceIds = cellSurfaceIdsList[planeIndex + 1];

                for (std::size_t index = 0; index != firstCellSurfaceIds->size(); ++index) {
                    if (firstCellSurfaceIds[index] < secondCellSurfaceIds[index]) {
                        break;
                    }
                    if (secondCellSurfaceIds[index] < firstCellSurfaceIds[index]) {
                        std::swap(firstCellSurfaceIds, secondCellSurfaceIds);
                        std::swap(firstCellSurfacePlane, secondCellSurfacePlane);
                        break;
                    }
                }
            }
        }
    }
    else if (relativeIdsByCellSurface.size() == 1) {
        auto surfaceRelativeIt = relativeIdsByCellSurface.begin();
        cellSurfacePlanes.resize(2);
        cellSurfaceIdsList.resize(2);

        if (surfaceRelativeIt->second[0] == 0 || pureDiagonalIndex == 0) {
            surfacePresenceList[0] = true;
            cellSurfacePlanes[0] = surfaceRelativeIt->first;
            cellSurfaceIdsList[0] = &surfaceRelativeIt->second;
        }
        else {
            surfacePresenceList[1] = true;
            cellSurfacePlanes[1] = surfaceRelativeIt->first;
            cellSurfaceIdsList[1] = &surfaceRelativeIt->second;
        }
    }

    auto newElementsStartingPosition = group.elements.size();

    std::size_t e = 0;

    for(std::size_t surfaceIndex = 0; surfaceIndex < cellSurfacePlanes.size(); ++surfaceIndex){
        if (surfacePresenceList[surfaceIndex]) {
            Element surface({}, Element::Type::Surface);
            surface.vertices.insert(surface.vertices.begin(), cellSurfaceIdsList[surfaceIndex]->begin(), cellSurfaceIdsList[surfaceIndex]->end());
            std::size_t firstCorrectOrientationIndex = 0;
            std::size_t differentAxes = 0;
            Cell firstCell;
            Cell secondCell;
            Cell thirdCell;

            do{
                ++firstCorrectOrientationIndex;
                firstCell = toCell(auxiliarMesh.coordinates[surface.vertices[firstCorrectOrientationIndex - 1]]);
                secondCell = toCell(auxiliarMesh.coordinates[surface.vertices[firstCorrectOrientationIndex]]);

                differentAxes = calculateDifferenceBetweenCells(firstCell, secondCell);

            } while(firstCorrectOrientationIndex < 4 && differentAxes != 1);

            thirdCell = toCell(auxiliarMesh.coordinates[surface.vertices[(firstCorrectOrientationIndex + 1) % 4]]);

            if(calculateDifferenceBetweenCells(secondCell, thirdCell) != 1){
                std::swap(surface.vertices[(firstCorrectOrientationIndex + 1) % 4], surface.vertices[(firstCorrectOrientationIndex + 2) % 4]);
            }

            group.elements.push_back(surface);
        }
    }
    if(relativeIdsByCellSurface.size() < 2){
        for(std::size_t e = 0; e < processedEdges.elements.size(); ++e){
            bool isInSurface = false;
            for (std::size_t surfaceIndex = 0; surfaceIndex < cellSurfacePlanes.size(); ++surfaceIndex){
                isInSurface = isInSurface
                    || surfacePresenceList[surfaceIndex]
                    && isEdgePartOfCellSurface(processedEdges.elements[e], *cellSurfaceIdsList[surfaceIndex]);
            }
            if (!isInSurface){
                group.elements.push_back(processedEdges.elements[e]);
            }
        }
    }

    for (std::size_t e = newElementsStartingPosition; e < group.elements.size(); ++e) {
        auto& element = group.elements[e];
        for (std::size_t v = 0; v < element.vertices.size(); ++v) {
            element.vertices[v] += newRelativeId;
        }
    }
}

bool Staircaser::isRelativeInCellsVector(const Relative& relative, const std::vector<Cell> & cells) const {
    Cell convertedCell = toCell(relative);

    for (auto& listCell : cells) {
        if (listCell == convertedCell) {
            return true;
        }
    }

    return false;
}

void Staircaser::filterSurfacesFromRelativeIds(
    const RelativeIds& triangleVertices,
    int pureDiagonalIndex,
    const Relatives& originalRelatives,
    const std::map<Surfel, IdSet>& idSetByCellSurface,
    Relatives& staircasedRelatives,
    std::map<Surfel, RelativeIds> & relativeIdsByCellSurface
) {
    std::vector<Cell> staircasedOriginalVertexCells;
    staircasedOriginalVertexCells.reserve(3);
    for (RelativeId v : triangleVertices) {
        staircasedOriginalVertexCells.push_back(calculateStaircasedCell(originalRelatives[v]));
    }

    auto cellSurfaceIt = idSetByCellSurface.begin();
    while (cellSurfaceIt != idSetByCellSurface.end()) {
        auto& plane = cellSurfaceIt->first;
        auto& idSet = cellSurfaceIt->second;
        auto numberOfSurfacePoints = idSet.size();


        std::vector<Cell> projectedCells;
        bool isCorrectSurface = false;

        if (pureDiagonalIndex >= 0 && numberOfSurfacePoints == 3) {
            for (auto& cell : staircasedOriginalVertexCells) {
                projectedCells.push_back(cell);
                projectedCells.back()[plane.second] = plane.first[plane.second];
            }

            isCorrectSurface = true;
            for (auto vertexIdIterator = idSet.begin(); isCorrectSurface && vertexIdIterator != idSet.end(); ++vertexIdIterator) {
                auto& surfaceRelative = staircasedRelatives[*vertexIdIterator];
                isCorrectSurface = isRelativeInCellsVector(surfaceRelative, projectedCells);
            }
        }

        if (numberOfSurfacePoints == 4 || (pureDiagonalIndex >= 0 && isCorrectSurface)) {
            relativeIdsByCellSurface[plane] = RelativeIds({});
            relativeIdsByCellSurface[plane].insert(relativeIdsByCellSurface[plane].begin(), idSet.begin(), idSet.end());
        }

        if (pureDiagonalIndex >= 0 && isCorrectSurface) {
            auto& surfaceIds = relativeIdsByCellSurface[plane];
            Cell missingCell = projectedCells[pureDiagonalIndex];
            for (auto relativeId : surfaceIds) {
                Cell surfaceCell = toCell(staircasedRelatives[relativeId]);
                auto differentAxes = calculateDifferentAxesBetweenCells(projectedCells[pureDiagonalIndex], surfaceCell);
                if (differentAxes.size() == 1) {
                    Axis axisToChange = X;

                    while (axisToChange == plane.second || axisToChange == differentAxes[0]) {
                        ++axisToChange;
                    }

                    missingCell[axisToChange] = projectedCells[(pureDiagonalIndex + 1) % 3][axisToChange];
                    break;
                }
            }
            for (auto relativeIt = surfaceIds.begin(); relativeIt != surfaceIds.end(); ++relativeIt) {
                Relative surfaceRelative = staircasedRelatives[*relativeIt];

                if (projectedCells[pureDiagonalIndex] == toCell(surfaceRelative)) {
                    auto positionToInsert = relativeIt + 1;
                    RelativeId missingRelativeId = staircasedRelatives.size();
                    staircasedRelatives.push_back(toRelative(missingCell));
                    surfaceIds.insert(positionToInsert, missingRelativeId);
                    break;
                }
            }
        }

        ++cellSurfaceIt;
    }
}

void Staircaser::addNewRelativeToGroupUsingBarycentre(const RelativeIds &triangleVertices, const Relatives &originalRelatives, Relatives &staircasedRelatives, Group &group)
{
    auto & firstTriangleVertex = originalRelatives[triangleVertices[0]];
    auto & secondTriangleVertex = originalRelatives[triangleVertices[1]];
    auto & thirdTriangleVertex = originalRelatives[triangleVertices[2]];

    Relative barycentre = (firstTriangleVertex + secondTriangleVertex + thirdTriangleVertex) / 3.0;

    Relatives missingPoints;
    missingPoints.reserve(2);

    std::set<Cell> cellSet;

    Cell minCell({
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max()
        });

    for(auto& staircasedRelative: staircasedRelatives){
        auto cell = toCell(staircasedRelative);

        for(Axis axis = X; axis <= Z; ++axis){
            minCell[axis] = std::min(minCell[axis], cell[axis]);
        }

        cellSet.insert(cell);

    }
    Cell cellCandidate;
    for(cellCandidate[X] = minCell[X]; cellCandidate[X] <= minCell[X] + 1 && missingPoints.size() != 2; ++cellCandidate[X]){
        for(cellCandidate[Y] = minCell[Y]; cellCandidate[Y] <= minCell[Y] + 1 && missingPoints.size() != 2; ++cellCandidate[Y]){
            for(cellCandidate[Z] = minCell[Z]; cellCandidate[Z] <= minCell[Z] + 1 && missingPoints.size() != 2; ++cellCandidate[Z]){
                if(cellSet.count(cellCandidate) != 1){
                    missingPoints.push_back(toRelative(cellCandidate));
                }
            }
        }
    }

    auto& firstPoint = missingPoints[0];
    auto& secondPoint = missingPoints[1];

    auto firstDistance = (barycentre - firstPoint).norm();
    auto secondDistance = (barycentre - secondPoint).norm();

    Relative newPoint;

    if(firstDistance <= secondDistance){
        newPoint = firstPoint;
    }
    else{
        newPoint = secondPoint;
    }

    RelativeId newId = staircasedRelatives.size();
    staircasedRelatives.push_back(newPoint);
}

void Staircaser::processLineAndAddToGroup(const Element& line, const Relatives& originalRelatives, Relatives& resultRelatives, Group& group) {
    auto startRelative = originalRelatives[line.vertices[0]];
    auto endRelative = originalRelatives[line.vertices[1]];

    RelativeId startIndex = resultRelatives.size();

    std::vector<Cell> cells = calculateMiddleCellsBetweenTwoRelatives(startRelative, endRelative);

    auto startRelativePosition = this->toRelative(cells.front());
    resultRelatives.push_back(startRelativePosition);

    if (cells.size() == 1) {
        group.elements.push_back(Element({ startIndex }, Element::Type::Node));
        return;
    }

    for (std::size_t v = 1; v < cells.size(); ++v) {
        auto endRelativePosition = this->toRelative(cells[v]);
        RelativeId endIndex = startIndex + 1;
        resultRelatives.push_back(endRelativePosition);

        group.elements.push_back(Element({ startIndex, endIndex }, Element::Type::Line));

        std::swap(startRelativePosition, endRelativePosition);
        ++startIndex;
    }
}

void Staircaser::processNodeAndAddToGroup(const Element& node, const Relatives& originalRelatives, Relatives& resultRelatives, Group& group) {
    auto relative = originalRelatives[node.vertices[0]];

    auto cell = calculateStaircasedCell(relative);
    auto cellRelativePosition = this->toRelative(cell);
    RelativeId index = resultRelatives.size();

    resultRelatives.push_back(cellRelativePosition);
    group.elements.push_back(Element({ index }, Element::Type::Node));
    return;
}

std:: vector<Cell> Staircaser::calculateMiddleCellsBetweenTwoRelatives(Relative& startExtreme, Relative& endExtreme) {
    // TODO: Compare with integers as substitute for floating point numbers with three decimals.

    auto startCell = this->calculateStaircasedCell(startExtreme);
    auto endCell = this->calculateStaircasedCell(endExtreme);
    auto startStaircased = this->toRelative(startCell);
    auto endStaircased = this->toRelative(endCell);

    std::vector<Cell> cells;
    cells.reserve(4);
    cells.push_back(startCell);
    

    Relative centerVector = startStaircased + (endStaircased - startStaircased) / 2.0;
    Relative distanceVector = endExtreme - startExtreme;
    Relative scaleVector;

    std::map<double, Relative> sortedIntersections;

    std::vector<Axis> differentAxes = calculateDifferentAxesBetweenCells(startCell, endCell);

    if (differentAxes.size() > 1) {
        for (Axis scaleAxis : differentAxes) {
            scaleVector[scaleAxis] = distanceVector[scaleAxis] / (centerVector[scaleAxis] - startExtreme[scaleAxis]);
            Relative componentPoint;

            for (Axis intersectionAxis = X; intersectionAxis <= Z; ++intersectionAxis) {
                if (intersectionAxis == scaleAxis) {
                    componentPoint[intersectionAxis] = centerVector[scaleAxis];
                }
                else {
                    componentPoint[intersectionAxis] = startExtreme[intersectionAxis] + distanceVector[intersectionAxis] / scaleVector[scaleAxis];
                }
            }

            double componentDistance = (componentPoint - startExtreme).norm();
            sortedIntersections[componentDistance] = componentPoint;
        }
        auto intersectionIt = sortedIntersections.begin();
        auto nextIntersectionIt = sortedIntersections.begin();
        ++nextIntersectionIt;
        while (intersectionIt != sortedIntersections.end()) {
            auto& point = intersectionIt->second;

            std::set<Axis> equalAxes;

            for (Axis currentAxis = X; currentAxis <= Z; ++currentAxis) {
                Axis firstAxis = currentAxis;
                Axis secondAxis = (firstAxis + 1) % 3;

                if (!approxDir(centerVector[firstAxis], startStaircased[firstAxis]) && !approxDir(centerVector[firstAxis], endStaircased[firstAxis])
                    && !approxDir(centerVector[secondAxis], startStaircased[secondAxis]) && !approxDir(centerVector[secondAxis], endStaircased[secondAxis])
                    && approxDir(centerVector[firstAxis], point[firstAxis]) && approxDir(centerVector[secondAxis], point[secondAxis])) {

                    if (endExtreme[firstAxis] < startExtreme[firstAxis]) {
                        firstAxis = 5 - firstAxis;
                    }
                    if (endExtreme[secondAxis] < startExtreme[secondAxis]) {
                        secondAxis = 5 - secondAxis;
                    }

                    equalAxes.insert(firstAxis);
                    equalAxes.insert(secondAxis);
                }
            }

            if (equalAxes.size() != 0) {
                Cell firstMiddleCell = cells.back();
                auto forcedAxisIt = equalAxes.begin();

                auto forcedAxis = *forcedAxisIt;

                if (forcedAxis > Z) {
                    forcedAxis = Z - (forcedAxis - 3);
                }

                firstMiddleCell[forcedAxis] = endCell[forcedAxis];
                ++forcedAxisIt;


                forcedAxis = *forcedAxisIt;

                if (forcedAxis > Z) {
                    forcedAxis = Z - (forcedAxis - 3);
                }

                Cell secondMiddleCell = firstMiddleCell;
                secondMiddleCell[forcedAxis] = endCell[forcedAxis];

                cells.push_back(firstMiddleCell);
                cells.push_back(secondMiddleCell);
            }
            else if (nextIntersectionIt != sortedIntersections.end()) {
                Relative& nextPoint = nextIntersectionIt->second;

                Relative middlePoint = (point + nextPoint) / 2;
                Cell middleCell = calculateStaircasedCell(middlePoint);
                cells.push_back(middleCell);
            }

            ++intersectionIt;
            if (nextIntersectionIt != sortedIntersections.end()) {
                ++nextIntersectionIt;
            }
        }
    }

    if (cells.back() != endCell) {
        cells.push_back(endCell);
    }

    return cells;
}

Cell Staircaser::calculateStaircasedCell(const Relative& relative) const
{
    auto resultCell = this->toCell(relative);

    for (std::size_t axis = 0; axis < 3; ++axis) {
        auto distance = relative[axis] - resultCell[axis];

        if (distance >= 0.5) {
            ++resultCell[axis];
        }
    }

    return resultCell;
}

std::size_t Staircaser::calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    short difference = 0;

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            ++difference;
        }
    }
    return difference;

}

std::vector<Axis> Staircaser::calculateDifferentAxesBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    std::vector<Axis> differentAxes;
    differentAxes.reserve(3);

    for (Axis axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] != secondCell[axis]) {
            differentAxes.push_back(axis);
        }
    }
    return differentAxes;
}

std::vector<Axis> Staircaser::calculateEqualAxesBetweenCells(const Cell& firstCell, const Cell& secondCell) {
    std::vector<Axis> equalAxes;
    equalAxes.reserve(3);

    for (Axis axis = 0; axis < 3; ++axis) {
        if (firstCell[axis] == secondCell[axis]) {
            equalAxes.push_back(axis);
        }
    }
    return equalAxes;
}

void Staircaser::calculateRelativeIdSetByCellSurface(const Relatives& relatives, std::map<Surfel, IdSet>& relativesByCellSurface) {
    Cell minCell({
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max(),
        std::numeric_limits<CellDir>::max()
        });


    for (auto & relative : relatives) {
        for (Axis axis = X; axis < 3; ++axis) {
            minCell[axis] = std::min(minCell[axis], toCellDir(relative[axis]));
        }
    }

    Cell maxCell = minCell;

    for (auto axis = X; axis <= Z; ++axis) {
        ++maxCell[axis];
    }

    for (RelativeId v = 0; v < relatives.size(); ++v) {
        Cell relativeCell = toCell(relatives[v]);
        auto minCellEqualAxes = calculateEqualAxesBetweenCells(minCell, relativeCell);
        auto maxCellEqualAxes = calculateEqualAxesBetweenCells(maxCell, relativeCell);

        for (auto axis : minCellEqualAxes) {
            relativesByCellSurface[{minCell, axis}].insert(v);
        }

        for (auto axis : maxCellEqualAxes) {
            relativesByCellSurface[{maxCell, axis}].insert(v);
        }
    }
}

bool Staircaser::isPureDiagonal(const Element& edge, const Relatives & relatives) {
    if (!edge.isLine()) {
        return false;
    }

    const auto& startPoint = relatives[edge.vertices[0]];
    const auto& endPoint = relatives[edge.vertices[1]];
    auto startCell = calculateStaircasedCell(startPoint);
    auto endCell = calculateStaircasedCell(endPoint);
    std::size_t difference = calculateDifferenceBetweenCells(startCell, endCell);

    if (difference != 3) {
        return false;
    }

    Relative startStaircased = toRelative(startCell);
    Relative endStaircased = toRelative(endCell);

    Relative centerVector = startStaircased + (endStaircased - startStaircased) / 2.0;
    Relative distanceVector = endPoint - startPoint;
    Relative scaleVector;
    
    for (Axis axis = X; axis <= Z; ++axis) {
        if(approxDir(centerVector[axis], startPoint[axis])){
            scaleVector[axis] = 0.0;
        }
        else{
            scaleVector[axis] = distanceVector[axis] / (centerVector[axis] - startPoint[axis]);
        }
    }

    if (!approxDir(scaleVector[X], scaleVector[Y]) || !approxDir(scaleVector[Y], scaleVector[Z])) {
        return false;
    }

    return true;
}

bool Staircaser::isEdgePartOfCellSurface(const Element& edge, const RelativeIds& surfaceRelativeIds) const {
    if (!edge.isLine()) {
        return false;
    }

    for (auto vertex : edge.vertices) {
        bool found = false;
        for (auto coordId : surfaceRelativeIds) {
            if (vertex == coordId) {
                found = true;
                break;
            }
        }

        if (!found) {
            return false;
        }
    }

    return true;
}
}
}
