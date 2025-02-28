#include "ConformalMesher.h"

#include "MesherBase.h"
#include "OffgridMesher.h"

#include "utils/GridTools.h"
#include "utils/CoordGraph.h"

namespace meshlib::meshers {

using namespace utils;
std::set<Cell> ConformalMesher::cellsWithMoreThanAVertexPerEdge(const Mesh& mesh)
{
    std::set<Cell> res;
    
    const GridTools gT(mesh.grid);
    std::set<std::pair<Cell, Axis>> ocuppiedEdges;
    for (const auto& v: mesh.coordinates) {
        if (gT.isRelativeOnCellEdge(v)) {
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
    
    auto gT = GridTools(mesh.grid);
    
    Elements allElements;
    for (auto const& g: mesh.groups) {
        allElements.insert(allElements.end(), g.elements.begin(), g.elements.end());
    }
    auto cellMap = gT.buildCellElemMap(allElements, mesh.coordinates);
    
    for (auto const& c: cellMap) {
        auto cG = CoordGraph(c.second);
        IdSet vIdsInCornerOrEdges;
        for (const auto& vId: cG.getVertices()) {
            const Relative& v = mesh.coordinates[vId];
            if (gT.isRelativeOnCellCorner(v) || gT.isRelativeOnCellEdge(v)) {
                vIdsInCornerOrEdges.insert(vId);
            }
        }

        // WIP
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
    // The five rules:
    // Rule #1: Cell edges must contain at most one vertex (ignoring cell corners).
    // Rule #2: Cell faces must always be crossed by a single path.

    std::set<Cell> res;

    res = mergeCellSets(res, cellsWithMoreThanAVertexPerEdge(mesh));
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

    // Find cells which break conformal rules.
    auto nonConformalCells = findNonConformalCells(res);

    // Calls structurer to mesh only those cells.
    

    return res;
}

}
