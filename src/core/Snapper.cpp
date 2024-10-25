#include "Snapper.h"

#include "utils/Geometry.h"
#include "utils/Cleaner.h"
#include "utils/MeshTools.h"
#include "cgal/Manifolder.h"
#include "Collapser.h"

namespace meshlib {
namespace tessellator {


Snapper::Snapper(const Mesh& mesh, const SnapperOptions& opts) :
    mesh_{ mesh },
    opts_{ opts }
{
    if (opts.forbiddenLength > 0.5) {
        throw std::logic_error("Invalid relaxed length");
    }
    snap();
    
    mesh_ = Collapser{mesh_, 4}.getMesh();
    
    utils::meshTools::checkNoCellsAreCrossed(mesh_);
    utils::meshTools::checkNoNullAreasExist(mesh_);
}

std::pair<Coordinates, std::map<Coordinate, std::set<LinV>>> Snapper::buildListOfValidSolverPoints() const
{
    // Vertices
    Coordinates v(8);
    v[0] = VecD({ 0.0, 0.0, 0.0 });
    v[1] = VecD({ 1.0, 0.0, 0.0 });
    v[2] = VecD({ 1.0, 1.0, 0.0 });
    v[3] = VecD({ 0.0, 1.0, 0.0 });
    v[4] = VecD({ 0.0, 0.0, 1.0 });
    v[5] = VecD({ 1.0, 0.0, 1.0 });
    v[6] = VecD({ 1.0, 1.0, 1.0 });
    v[7] = VecD({ 0.0, 1.0, 1.0 });

    // Edges
    std::vector<LinV> edges(12);
    edges[0] = { v[0], v[1] };
    edges[1] = { v[1], v[2] };
    edges[2] = { v[2], v[3] };
    edges[3] = { v[3], v[0] };
    edges[4] = { v[0], v[4] };
    edges[5] = { v[1], v[5] };
    edges[6] = { v[2], v[6] };
    edges[7] = { v[3], v[7] };
    edges[8] = { v[4], v[5] };
    edges[9] = { v[5], v[6] };
    edges[10] = { v[6], v[7] };
    edges[11] = { v[7], v[4] };

    std::map<Coordinate, std::set<LinV>> coordsToEdge;

    std::set<Coordinate> aux;
    for (auto const& e : edges) {
        const Coordinate& vIni = e[0];
        aux.insert(vIni);
        coordsToEdge[vIni].insert(e);
        const Coordinate& vEnd = e[1];
        aux.insert(vEnd);
        coordsToEdge[vEnd].insert(e);

        const Coordinate vRIni = vIni + (vEnd - vIni) * opts_.forbiddenLength;
        aux.insert(vRIni);
        coordsToEdge[vRIni].insert(e);
        const Coordinate vREnd = vIni + (vEnd - vIni) * (1.0 - opts_.forbiddenLength);
        aux.insert(vREnd);
        coordsToEdge[vREnd].insert(e);

        for (std::size_t i = 0; i < opts_.edgePoints; i++) {
            const double t = double(i + 1) / double(opts_.edgePoints + 1);
            const Coordinate inner = vRIni + (vREnd - vRIni) * t;
            aux.insert(inner);
            coordsToEdge[inner].insert(e);

        }
    }

    return std::make_pair(Coordinates(aux.begin(), aux.end()), coordsToEdge);
}

bool edgeIsCandidate(
    const LinV& edge,
    const Relative& rel,
    const utils::GridTools& gT)
{
    if (gT.isRelativeOnCellFace(rel)) {
        const Axis& axis = gT.getCellFaceAxis(rel).second;
        return !(edge[0][axis] == 1 || edge[1][axis] == 1);
    }
    return true;
}


void Snapper::snap()
{
    Coordinates solverPoints;
    std::map<Coordinate, std::set<LinV>> coordsToEdge;
    std::tie(solverPoints, coordsToEdge) = buildListOfValidSolverPoints();
    // Snaps using closest point
    utils::GridTools gT(mesh_.grid);
    std::vector<Relative> r = mesh_.coordinates;

    std::map<LinV, Coordinates> edgeToSnappedCoords;

    for (std::size_t i = 0; i < r.size(); i++) {
        Relative rel = r[i];
        if (gT.isRelativeOnCellEdge(rel)) {
            Coordinate closest, solverPoint;
            std::tie(closest, solverPoint) = findClosestSolverPoint(rel, solverPoints, gT);

            CoordinateId id = i;
            r[id] = closest;

            VecD cell = gT.toCell(rel).as<double>();
            for (const auto& edge : coordsToEdge[solverPoint]) {
                LinV l = { edge[0] + cell, edge[1] + cell };
                edgeToSnappedCoords[l].push_back(closest);
            }
        }
    }

    for (std::size_t i = 0; i < r.size(); i++) {
        Relative rel = r[i];
        if (!gT.isRelativeOnCellEdge(rel) && !gT.isRelativeOnCellCorner(rel)) {
            Coordinate closest, solverPoint;
            std::tie(closest, solverPoint) = findClosestSolverPoint(rel, solverPoints, gT);

            VecD cell = gT.toCell(r[i]).as<double>();
            double minDist = std::numeric_limits<double>::max();
            for (const auto& edge : coordsToEdge[solverPoint]) {
                if (edgeIsCandidate(edge, rel, gT)) {
                    LinV l = { edge[0] + cell, edge[1] + cell };
                    for (const auto& c : edgeToSnappedCoords[l]) {
                        double dist = (gT.getPos(rel) - gT.getPos(c)).norm();
                        if (dist < minDist) {
                            minDist = dist;
                            closest = c;
                        }
                    }
                }
            }
            CoordinateId id = i;
            r[id] = closest;
        }
    }

    mesh_.coordinates = r;

 }

std::pair<Coordinate, Coordinate> Snapper::findClosestSolverPoint(
    const Relative& rel,
    const Coordinates& solverPoints,
    const utils::GridTools& gT) const
{
    Coordinate closest;
    double minDist = std::numeric_limits<double>::max();
    std::size_t minEdge = 0;
    Coordinate pos = gT.getPos(rel);
    for (std::size_t j = 0; j < solverPoints.size(); j++) {
        Relative cellGridPoint = solverPoints[j] + gT.toCell(rel).as<double>();
        double dist = (pos - gT.getPos(cellGridPoint)).norm();
        if (dist < minDist) {
            minDist = dist;
            closest = cellGridPoint;
            minEdge = j;
        }
    }
    return std::make_pair(closest, solverPoints[minEdge]);
}



}
}