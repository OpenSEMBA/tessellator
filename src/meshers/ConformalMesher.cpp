#include "ConformalMesher.h"

#include "MesherBase.h"
#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Smoother.h" 
#include "core/Snapper.h"

#include "utils/GridTools.h"
#include "utils/MeshTools.h"
#include "utils/CoordGraph.h"

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;

std::set<Cell> ConformalMesher::cellsWithMoreThanAVertexInsideEdge(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const GridTools gT(mesh.grid);
    std::set<std::pair<Cell, Axis>> ocuppiedEdges;
    for (const auto& v: mesh.coordinates) {
        if (gT.isRelativeInCellEdge(v)) {
            auto edge = std::make_pair(
                gT.toCell(v), 
                gT.getCellEdgeAxis(v).second);
            if (ocuppiedEdges.count(edge)) {
                auto touchingCells = gT.getTouchingCells(v);
                res.insert(touchingCells.begin(), touchingCells.end());
            } else {
                ocuppiedEdges.insert(edge);
            }
        }
    }

    return res;
}

std::map<Cell, std::vector<const Element*>> buildCellMapForAllElements(const Mesh& mesh)
{
    const auto gT = GridTools(mesh.grid);
    std::map<Cell, std::vector<const Element*>> res;
    for (auto const& g: mesh.groups) {
        auto groupElemMap = gT.buildCellElemMap(g.elements, mesh.coordinates);
        for (auto const& cellElemPair: groupElemMap) {
            res[cellElemPair.first].insert(
                res[cellElemPair.first].end(),
                cellElemPair.second.begin(),
                cellElemPair.second.end());
        }
    }
    return res;
}

std::size_t countPathsInCellBound(
    const GridTools& gT,
    const Mesh& mesh,
    const Cell& cell,
    const ElementsView& elementsInCell,
    const std::pair<Axis, Side>& bound)
{
    auto cG = CoordGraph(elementsInCell);
    auto vIds = cG.getVertices();
        
    std::size_t pathsInCellBound = 0;

    // Get vertices in the cell bound.
    IdSet vIdsInBound;
    for (auto const& vId: vIds) {
        if (gT.isRelativeAtCellBound(
            mesh.coordinates[vId], cell, bound)) {
            vIdsInBound.insert(vId);
        }
    }
    if (vIdsInBound.size() == 0) {
        return pathsInCellBound;
    }

    // Keep only edges if both vertices are in the cell bound.
    // Isolated vertices are not included.
    // Edges between two triangles are removed.
    CoordGraph cellBoundGraph;
    for (const auto& line: cG.getEdgesAsLines()) {
        const auto& v0 = line.vertices[0];
        const auto& v1 = line.vertices[1];
        if (!vIdsInBound.count(v0) || !vIdsInBound.count(v1)) {
            continue;
        }
        cellBoundGraph.addEdge(v0,v1);
    }
    Elements graphEdges = cellBoundGraph.getBoundaryGraph().getEdgesAsLines();
       
    // Count non-edge lines pointing outwards 
    // from each vertex in the edge.
    for (const auto& line: graphEdges) {
        const auto& c0 = mesh.coordinates[line.vertices[0]];
        const auto& c1 = mesh.coordinates[line.vertices[1]];
        if (!GridTools::areCoordOnSameEdge(c0, c1)) {
            pathsInCellBound++;
        }
    }
    return pathsInCellBound;
}

std::set<Cell> ConformalMesher::cellsWithMoreThanAPathPerFace(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const auto gT = GridTools(mesh.grid);
    
    for (auto const& c: buildCellMapForAllElements(mesh)) {
        for (Axis x: {X, Y, Z}) {
            for (Side s: {L, U}) {
                auto pathsInCellBound = countPathsInCellBound(gT, mesh, c.first, c.second, {x,s});
                if (pathsInCellBound > 1) {
                    res.insert(c.first);
                    Cell adjacentCell = c.first;
                    adjacentCell[x] = c.first[x] + (s == L ? -1 : 1);
                    res.insert(adjacentCell);
                }
            }
        }
    }
 
    return res;
}

std::set<Cell> ConformalMesher::cellsWithAVertexInAnEdgeForbiddenRegion(const Mesh& mesh)
{
    std::set<Cell> res;
    // TODO
    return res;
}

std::set<Cell> mergeCellSets(const std::set<Cell>& a, const std::set<Cell>& b)
{
    std::set<Cell> res;
    res.insert(a.begin(), a.end());
    res.insert(b.begin(), b.end());
    return res;
}   

std::set<Cell> ConformalMesher::findNonConformalCells(const Mesh& mesh)
{
    // Find cells not respecting **The Three Rules**.
    std::set<Cell> res;
    
    // Rule #1: Cell edges must contain at most one vertex in each edge.
    res = mergeCellSets(res, cellsWithMoreThanAVertexInsideEdge(mesh));
    
    // Rule #2: Cell faces must always be crossed by a single path.
    res = mergeCellSets(res, cellsWithMoreThanAPathPerFace(mesh));

    // Rule #3: Conformal cells can't contain node or line elements.
    // res = mergeCellSets(res, cellsContainingNodeOrLineElements(mesh));

    return res;
}

Mesh ConformalMesher::mesh() const 
{
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    
    auto res = inputMesh_;

    if (res.countElems() == 0) {
        res.grid = slicingGrid;
        return res;
    }
    
    log("Slicing.", 1);
    res.grid = slicingGrid;
    res = Slicer{ res }.getMesh();
        
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    log("Smoothing.", 1);
    SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
    res = Smoother{res, smootherOpts}.getMesh();
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));
    
    log("Snapping.", 1);
    res = Snapper(res, opts_.snapperOptions).getMesh();
    logNumberOfTriangles(countMeshElementsIf(res, isTriangle));

    // Find cells which break conformal FDTD rules.
    auto nonConformalCells = findNonConformalCells(res);
    log("Non-conformal cells found: " + std::to_string(nonConformalCells.size()), 1);

    // Calls structurer to mesh only those cells.
    
    // Merges triangles which are on same cell face.
    
    
    reduceGrid(res, originalGrid_);
    
    // Converts relatives to absolutes.
    utils::meshTools::convertToAbsoluteCoordinates(res);


    return res;
}

}
