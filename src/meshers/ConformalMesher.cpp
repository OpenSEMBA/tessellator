#include "ConformalMesher.h"

#include "MesherBase.h"
#include "OffgridMesher.h"

#include "utils/GridTools.h"
#include "utils/CoordGraph.h"

namespace meshlib::meshers {

using namespace utils;
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

std::set<Cell> ConformalMesher::cellsWithMoreThanAPathPerFace(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const auto gT = GridTools(mesh.grid);
    
    Elements allElements; 
    for (auto const& g: mesh.groups) {
        allElements.insert(allElements.end(), g.elements.begin(), g.elements.end());
    }
    auto cellMap = gT.buildCellElemMap(allElements, mesh.coordinates);
    
    for (auto const& c: cellMap) {
        auto cG = CoordGraph(c.second);
        auto vIds = cG.getVertices();
        for (Axis x: {X, Y, Z}) {
            for (Side s: {L, U}) {
                // Get vertices in the cell bound.
                IdSet vIdsInBound;
                for (auto const& vId: vIds) {
                    if (gT.isRelativeAtCellBound(
                        mesh.coordinates[vId], c.first, {x,s})) {
                        vIdsInBound.insert(vId);
                    }
                }
                if (vIdsInBound.size() == 0) {
                    continue;
                }

                // Keep only edges if both vertices are in the cell bound.
                // Isolated vertices are not included.
                CoordGraph faceGraph;
                for (auto const& line: cG.getEdgesAsLines()) {
                    const auto& v1 = line.vertices[0];
                    const auto& v2 = line.vertices[1];
                    if (vIdsInBound.count(v1) && vIdsInBound.count(v2)) {
                        faceGraph.addEdge(v1, v2);
                    }
                }
               
                // Count non-edge-aligned lines pointing outwards 
                // from each vertex in the edge.
                auto lines = faceGraph.getEdgesAsLines();
                std::size_t pathsInFace = 0;
                for (auto const& vId: vIdsInBound) {
                    if (!gT.isRelativeInCellEdge(mesh.coordinates[vId])) {
                        continue;
                    }
                    for (const auto& line: lines) {
                        if (line.vertices[0] == vId &&
                            !GridTools::areCoordOnSameEdge(
                                mesh.coordinates[line.vertices[0]],
                                mesh.coordinates[line.vertices[1]])) {
                            pathsInFace++;
                        }
                    }
                }

                if (pathsInFace > 1) {
                    res.insert(c.first);
                }
            }
        }
    }
 
    return res;
}

std::set<Cell> cellsWithInteriorDisconnectedTriangles(const Mesh& mesh)
{
    // Triangles inside cells must be part of a patch with at least one valid line
    // on a cell face.
    std::set<Cell> res;
    // TODO
    return res;
}

std::set<Cell> cellsWithVertexInForbiddenEdgeRegion(const Mesh& mesh)
{
    // All vertices must be out of the edge`s forbidden regions.
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
    // The five rules.
    std::set<Cell> res;
    
    // Rule #1: Cell edges must contain at most one vertex (ignoring cell corners).
    res = mergeCellSets(res, cellsWithMoreThanAVertexInsideEdge(mesh));
    
    // Rule #2: Cell faces must always be crossed by a single path.
    res = mergeCellSets(res, cellsWithMoreThanAPathPerFace(mesh));

    // res = mergeCellSets(res, cellsWithInteriorDisconnectedTriangles(mesh));
    // res = mergeCellSets(res, cellsWithVertexInForbiddenEdgeRegion(mesh));
    // res = mergeCellSets(res, cellsContainingLineElements(mesh));

    return res;
}

Mesh ConformalMesher::mesh() const 
{
    OffgridMesherOptions offgridMesherOpts;
    offgridMesherOpts.snapperOptions = opts_.snapperOptions;
    offgridMesherOpts.decimalPlacesInCollapser = opts_.decimalPlacesInCollapser;    

    Mesh res = OffgridMesher(inputMesh_, offgridMesherOpts).mesh();

    // Find cells which break conformal FDTD rules.
    auto nonConformalCells = findNonConformalCells(res);

    // Calls structurer to mesh only those cells.
    
    // Merges triangles which are on same cell face.

    return res;
}

}
