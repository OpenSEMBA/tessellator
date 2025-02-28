#include "ConformalMesher.h"

#include "MesherBase.h"
#include "OffgridMesher.h"

namespace meshlib::meshers {

std::set<Cell> cellsWithMoreThanAVertexPerEdge(const Mesh& mesh)
{
    // Cell edges can contain at most one vertex.
    std::set<Cell> res;
    // TODO
    return res;
}

std::set<Cell> cellsWithMoreThanALinePerFace(const Mesh& mesh)
{
    // Cell faces must always be crossed by a single line.
    std::set<Cell> res;
    // TODO
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

std::set<Cell> ConformalMesher::findNonConformalCells(const Mesh& mesh) const
{
    std::set<Cell> res;

    res = mergeCellSets(res, cellsWithMoreThanAVertexPerEdge(mesh));
    // res = mergeCellSets(res, cellsWithMoreThanALinePerFace(mesh));
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
