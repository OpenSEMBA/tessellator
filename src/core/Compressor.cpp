#include "Compressor.h"

#include <algorithm>
#include <queue>

#include "utils/Geometry.h"
#include "utils/GridTools.h"

namespace meshlib::core {

using meshlib::Sign;
using meshlib::PlanePoint;
using meshlib::PlaneLinel;
using meshlib::PlaneSurfel;
using meshlib::PlaneSurface;
using meshlib::Contour;
using meshlib::CrossLine;

std::size_t Compressor::compressSurfaces(Mesh& mesh) {
    std::size_t totalOriginal = 0;
    std::size_t totalCompressed = 0;

    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        std::vector<Element> surfs;
        std::vector<ElementId> surfIndices;
        
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            const Element& elem = mesh.groups[g].elements[e];
            if (elem.type == Element::Type::Surface) {
                surfIndices.push_back(e);
                surfs.push_back(elem);
            }
        }

        if (surfs.empty()) {
            continue;
        }

        totalOriginal += surfs.size();
        std::vector<Element> compressedSurfs = compressSurfs_(mesh.coordinates, surfs);
        totalCompressed += compressedSurfs.size();

        // Build new elements vector with compressed surfaces
        std::vector<Element> newElements;
        ElementId surfIdx = 0;
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            if (mesh.groups[g].elements[e].type == Element::Type::Surface) {
                if (surfIdx < compressedSurfs.size()) {
                    newElements.push_back(compressedSurfs[surfIdx]);
                    surfIdx++;
                }
            } else {
                newElements.push_back(mesh.groups[g].elements[e]);
            }
        }
        mesh.groups[g].elements = std::move(newElements);
    }

    return totalOriginal - totalCompressed;
}

std::vector<Element> Compressor::compressSurfs_(
        std::vector<Coordinate>& coords,
        const std::vector<Element>& surfs) {
    std::vector<Element> res;
    std::map<std::pair<CellDir, std::pair<Sign, Axis>>,
             std::vector<ElementId>> signDirSurfs;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        if (surfs[s].vertices.size() != 4) {
            res.push_back(surfs[s]);
            continue;
        }
        std::array<Cell, 3> auxCells;
        auxCells[0] = utils::GridTools::toCell(coords[surfs[s].vertices[0]]);
        auxCells[1] = utils::GridTools::toCell(coords[surfs[s].vertices[1]]);
        auxCells[2] = utils::GridTools::toCell(coords[surfs[s].vertices[2]]);
        CellDir gridSurf;
        Sign sign = 1;
        Axis dir = 0;
        for (Axis d = 0; d < 3; d++) {
            if (auxCells[0](d) == auxCells[2](d)) {
                Cell normal = (auxCells[1] - auxCells[0]) ^
                              (auxCells[2] - auxCells[0]);
                if (normal(d) >= 0) {
                    sign = 1;
                } else {
                    sign = -1;
                }
                dir = d;
                gridSurf = auxCells[0](d);
                break;
            }
        }
        signDirSurfs[std::make_pair(gridSurf,
                                    std::make_pair(sign, dir))].push_back(s);
    }
    for (std::map<std::pair<CellDir, std::pair<Sign, Axis>>,
                  std::vector<ElementId>>::const_iterator
         it = signDirSurfs.begin(); it != signDirSurfs.end(); ++it) {
        std::vector<Element> auxElems;
        for (std::size_t i = 0; i < it->second.size(); i++) {
            auxElems.push_back(surfs[it->second[i]]);
        }
        std::vector<Element> auxRes =
            compressDirSignSurfs_(coords,
                                  it->first.second,
                                  auxElems);
        res.insert(res.end(), auxRes.begin(), auxRes.end());
    }
    return res;
}

std::vector<Element> Compressor::compressDirSignSurfs_(
        std::vector<Coordinate>& coords,
        const std::pair<Sign, Axis>& signDir,
        const std::vector<Element>& surfs) {
    std::vector<Element> res;
    std::map<LinIds, std::set<ElementId>> lineSurfs;
    std::map<ElementId, std::set<LinIds>> surfLines;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        for (std::size_t i = 0; i < 4; i++) {
            std::size_t j = (i + 1) % 4;
            LinIds line;
            line[0] = surfs[s].vertices[i];
            line[1] = surfs[s].vertices[j];
            std::sort(line.begin(), line.end());
            lineSurfs[line].insert(s);
            surfLines[s].insert(line);
        }
    }
    std::set<ElementId> vis;
    for (std::map<ElementId, std::set<LinIds>>::const_iterator
         itExt = surfLines.begin(); itExt != surfLines.end(); ++itExt) {
        if (vis.count(itExt->first) == 0) {
            std::set<ElementId> surfsConn;
            std::queue<ElementId> q;
            q.push(itExt->first);
            vis.insert(itExt->first);
            while (!q.empty()) {
                ElementId elem = q.front();
                q.pop();
                surfsConn.insert(elem);
                for (std::set<LinIds>::const_iterator
                     itLine  = surfLines[elem].begin();
                     itLine != surfLines[elem].end(); ++itLine) {
                    for (std::set<ElementId>::const_iterator
                         itSurf  = lineSurfs[*itLine].begin();
                         itSurf != lineSurfs[*itLine].end(); ++itSurf) {
                        if (vis.count(*itSurf) == 0) {
                            q.push(*itSurf);
                            vis.insert(*itSurf);
                        }
                    }
                }
            }
            std::vector<Element> resCon;
            for (std::set<ElementId>::const_iterator
                 it = surfsConn.begin(); it != surfsConn.end(); ++it) {
                resCon.push_back(surfs[*it]);
            }
            resCon = compressSurf_(coords, signDir, resCon);
            res.insert(res.end(), resCon.begin(), resCon.end());
        }
    }
    return res;
}

std::vector<Element> Compressor::compressSurf_(
        std::vector<Coordinate>& coords,
        const std::pair<Sign, Axis>& signDir,
        const std::vector<Element>& surfs) {
    std::vector<Element> res;
    Axis d = signDir.second;
    Axis d1 = (d + 1) % 3;
    Axis d2 = (d + 2) % 3;
    CellDir plane = utils::GridTools::toCell(coords[surfs[0].vertices[0]])(d);
    std::set<PlaneSurfel> surfels;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        std::pair<PlanePoint, PlanePoint> ext;
        ext.first[0] =
            utils::GridTools::toCell(coords[surfs[s].vertices[0]])(d1);
        ext.first[1] =
            utils::GridTools::toCell(coords[surfs[s].vertices[0]])(d2);
        ext.second[0] =
            utils::GridTools::toCell(coords[surfs[s].vertices[2]])(d1);
        ext.second[1] =
            utils::GridTools::toCell(coords[surfs[s].vertices[2]])(d2);
        for (CellDir i = ext.first[0]; i < ext.second[0]; i++) {
            for (CellDir j = ext.first[1]; j < ext.second[1]; j++) {
                PlaneSurfel surfel = {{i, j}};
                surfels.insert(surfel);
            }
        }
    }
    std::vector<PlaneSurface> aux = compressSurfels_(surfels);
    CoordinateMap coordMap;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        for (std::size_t i = 0; i < 4; i++) {
            CoordinateId coordId = surfs[s].vertices[i];
            Coordinate   coord   = coords[coordId];
            coordMap[coord] = coordId;
        }
    }
    for (std::size_t e = 0; e < aux.size(); e++) {
        std::array<Cell, 4> ext;
        ext[0](d) = ext[2](d) = plane;
        ext[0](d1) = aux[e].first[0];
        ext[0](d2) = aux[e].first[1];
        ext[2](d1) = aux[e].second[0];
        ext[2](d2) = aux[e].second[1];
        ext[1] = ext[3] = ext[0];
        ext[1](d1) = ext[2](d1);
        ext[3](d2) = ext[2](d2);
        if (signDir.first < 0) {
            std::swap(ext[1], ext[3]);
        }
        Element resElem;
        resElem.type = Element::Type::Surface;
        for (std::size_t i = 0; i < 4; i++) {
            Relative rel = utils::GridTools::toRelative(ext[i]);
            if (coordMap.count(rel) == 0) {
                coordMap[rel] = coords.size();
                coords.push_back(rel);
            }
            resElem.vertices.push_back(coordMap[rel]);
        }
        res.push_back(resElem);
    }
    return res;
}

std::vector<PlaneSurface> Compressor::compressSurfels_(
        const std::set<PlaneSurfel>& surfs) {
    std::vector<PlaneSurface> res;
    const std::vector<Contour>& conts = getContours_(surfs);
    std::array<std::vector<CrossLine>, 2> cross =
        getCrossingLines_(surfs, conts);
    cross = getMaxCompatLines_(cross);
    std::set<PlaneLinel> linels;
    for (std::size_t c = 0; c < conts.size(); c++) {
        for (std::size_t i = 0; i < conts[c].size(); i++) {
            std::size_t j = (i + 1) % conts[c].size();
            std::set<PlaneLinel> aux = getLinels_(conts[c][i], conts[c][j]);
            linels.insert(aux.begin(), aux.end());
        }
    }
    for (Axis d = 0; d < 2; d++) {
        Axis d1 = (d + 1) % 2;
        for (std::size_t i = 0; i < cross[d].size(); i++) {
            PlanePoint ini, end;
            ini[d1] = end[d1] = cross[d][i].first;
            ini[d] = cross[d][i].second.first;
            end[d] = cross[d][i].second.second;
            std::set<PlaneLinel> aux = getLinels_(ini, end);
            linels.insert(aux.begin(), aux.end());
        }
    }
    addConcaveLinels_(surfs, conts, linels);
    std::map<PlaneLinel, std::set<PlaneSurfel>> lineSurfs;
    std::map<PlaneSurfel, std::set<PlaneLinel>> surfLines;
    for (std::set<PlaneSurfel>::const_iterator
         it = surfs.begin(); it != surfs.end(); ++it) {
        surfLines.insert(std::make_pair(*it, std::set<PlaneLinel>()));
        for (Axis d = 0; d < 2; d++) {
            for (CellDir diff = -1; diff <= 1; diff += 2) {
                PlaneLinel linel = getSurfaceEdge_(*it, diff, d);
                if (linels.count(linel) == 0) {
                    surfLines[*it].insert(linel);
                    lineSurfs[linel].insert(*it);
                }
            }
        }
    }
    std::set<PlaneSurfel> vis;
    for (std::map<PlaneSurfel, std::set<PlaneLinel>>::const_iterator
         itSurfExt = surfLines.begin();
         itSurfExt != surfLines.end(); ++itSurfExt) {
        if (vis.count(itSurfExt->first) == 0) {
            std::queue<PlaneSurfel> q;
            q.push(itSurfExt->first);
            vis.insert(itSurfExt->first);
            PlanePoint minPoint = itSurfExt->first;
            PlanePoint maxPoint = itSurfExt->first;
            while (!q.empty()) {
                PlaneSurfel surfel = q.front();
                q.pop();
                if (surfel < minPoint) {
                    minPoint = surfel;
                }
                if (surfel > maxPoint) {
                    maxPoint = surfel;
                }
                for (std::set<PlaneLinel>::const_iterator
                     itLin = surfLines[surfel].begin();
                     itLin != surfLines[surfel].end(); ++itLin) {
                    for (std::set<PlaneSurfel>::const_iterator
                         itSurfInt = lineSurfs[*itLin].begin();
                         itSurfInt != lineSurfs[*itLin].end(); ++itSurfInt) {
                        if (vis.count(*itSurfInt) == 0) {
                            q.push(*itSurfInt);
                            vis.insert(*itSurfInt);
                        }
                    }
                }
            }
            maxPoint[0]++;
            maxPoint[1]++;
            res.push_back(std::make_pair(minPoint, maxPoint));
        }
    }
    return res;
}

std::vector<Contour> Compressor::getContours_(
        const std::set<PlaneSurfel>& surfs) {
    std::vector<Contour> res;
    if (surfs.empty()) {
        return res;
    }
    std::set<PlaneLinel> vis;
    res.push_back(
        getContour_(getSurfaceEdge_(*surfs.begin(), -1, 0), surfs, vis));
    for (std::set<PlaneSurfel>::const_iterator
         it = surfs.begin(); it != surfs.end(); ++it) {
        for (Axis d = 0; d < 2; d++) {
            for (CellDir diff = -1; diff <= 1; diff += 2) {
                PlaneSurfel adjSurf;
                PlaneLinel adjEdge;
                adjSurf = *it;
                adjSurf[d] += diff;
                adjEdge = getSurfaceEdge_(*it, diff, d);
                if ((surfs.find(adjSurf) == surfs.end()) &&
                    (vis.find(adjEdge) == vis.end())) {
                    res.push_back(getContour_(adjEdge, surfs, vis));
                }
            }
        }
    }
    return res;
}

Contour Compressor::getContour_(
        const PlaneLinel& from,
        const std::set<PlaneSurfel>& surfs,
        std::set<PlaneLinel>& vis) {
    Contour res;
    std::queue<PlaneLinel> q;
    if (vis.find(from) != vis.end()) {
        return res;
    }
    std::vector<PlaneLinel> lines;
    q.push(from);
    vis.insert(from);
    lines.push_back(from);
    while (!q.empty()) {
        PlaneLinel edge = q.front();
        q.pop();
        PlanePoint pos = edge.first;
        Axis d0 = edge.second;
        Axis d1 = (d0 + 1) % 2;
        PlaneSurfel surf = pos;
        if (surfs.find(surf) == surfs.end()) {
            surf[d1]--;
        }
        for (CellDir diff = -1; diff <= 1; diff += 2) {
            PlaneSurfel adjSurf1 = surf;
            adjSurf1[d0] += diff;
            if (surfs.find(adjSurf1) == surfs.end()) {
                PlaneLinel adjEdge = getSurfaceEdge_(surf, diff, d0);
                if (vis.find(adjEdge) == vis.end()) {
                    q.push(adjEdge);
                    vis.insert(adjEdge);
                    lines.push_back(adjEdge);
                    break;
                }
                continue;
            }
            if (surf == pos) {
                adjSurf1[d1]--;
            } else {
                adjSurf1[d1]++;
            }
            if (surfs.find(adjSurf1) == surfs.end()) {
                PlaneLinel adjEdge = edge;
                adjEdge.first[d0] += diff;
                if (vis.find(adjEdge) == vis.end()) {
                    q.push(adjEdge);
                    vis.insert(adjEdge);
                    lines.push_back(adjEdge);
                    break;
                }
                continue;
            } else {
                PlaneLinel adjEdge = getSurfaceEdge_(adjSurf1, -diff, d0);
                if (vis.find(adjEdge) == vis.end()) {
                    q.push(adjEdge);
                    vis.insert(adjEdge);
                    lines.push_back(adjEdge);
                    break;
                }
                continue;
            }
        }
    }
    for (std::vector<PlaneLinel>::const_iterator
         it = lines.begin(); it != lines.end(); ++it) {
        std::vector<PlaneLinel>::const_iterator itPlus = std::next(it);
        if (itPlus == lines.end()) {
            itPlus = lines.begin();
        }
        if (it->second == itPlus->second) {
            continue;
        }
        std::array<PlanePoint, 2> extremes = {it->first, it->first};
        extremes[1][it->second]++;
        std::array<PlanePoint, 2> extremesP = {itPlus->first, itPlus->first};
        extremesP[1][itPlus->second]++;
        for (std::size_t p = 0; p < 4; p++) {
            if (extremes[p / 2] == extremesP[p % 2]) {
                res.push_back(extremes[p / 2]);
                break;
            }
        }
    }
    return res;
}

PlaneLinel Compressor::getSurfaceEdge_(const PlaneSurfel& surf,
                                                   const CellDir& diff,
                                                   const Axis& dir) {
    PlaneLinel res;
    res.first  = surf;
    res.second = (dir + 1) % 2;
    if (diff > 0) {
        res.first[dir]++;
    }
    return res;
}

std::array<std::vector<CrossLine>, 2>
    Compressor::getCrossingLines_(
        const std::set<PlaneSurfel>& surfs,
        const std::vector<Contour>& conts) {
    std::array<std::vector<CrossLine>, 2> res;
    for (Axis d = 0; d < 2; d++) {
        Axis d1 = (d + 1) % 2;
        std::map<CellDir, std::set<CellDir>> cross;
        for (std::vector<Contour>::const_iterator
             it1 = conts.begin(); it1 != conts.end(); ++it1) {
            for (std::vector<PlanePoint>::const_iterator
                 it2 = it1->begin(); it2 != it1->end(); ++it2) {
                cross[(*it2)[d1]].insert((*it2)[d]);
            }
        }
        for (std::map<CellDir, std::set<CellDir>>::const_iterator
             itMap = cross.begin(); itMap != cross.end(); ++itMap) {
            for (std::set<CellDir>::const_iterator
                 itSet = itMap->second.begin();
                 itSet != itMap->second.end(); ++itSet) {
                std::set<CellDir>::const_iterator itSetPlus = std::next(itSet);
                if (itSetPlus == itMap->second.end()) {
                    break;
                }
                bool valid = true;
                PlanePoint edge;
                edge[d1] = itMap->first;
                for (CellDir i = *itSet; i < *itSetPlus; i++) {
                    edge[d] = i;
                    PlaneSurfel adjSurf1, adjSurf2;
                    adjSurf1 = adjSurf2 = edge;
                    adjSurf1[d1]--;
                    if ((surfs.find(adjSurf1) == surfs.end()) ||
                        (surfs.find(adjSurf2) == surfs.end())) {
                        valid = false;
                        break;
                    }
                }
                if (valid) {
                    res[d].push_back(
                        std::make_pair(itMap->first,
                                       std::make_pair(*itSet, *itSetPlus)));
                }
            }
        }
    }
    return res;
}

std::array<std::vector<CrossLine>, 2>
    Compressor::getMaxCompatLines_(
        const std::array<std::vector<CrossLine>, 2>& cross) {
    std::array<std::vector<CrossLine>, 2> res;
    if (cross[0].size() > cross[1].size()) {
        res[0] = cross[0];
    } else {
        res[1] = cross[1];
    }
    return res;
}

std::set<PlaneLinel> Compressor::getLinels_(
        const PlanePoint& ini,
        const PlanePoint& end) {
    std::set<PlaneLinel> res;
    for (Axis d = 0; d < 2; d++) {
        Axis d1 = (d + 1) % 2;
        if (ini[d] == end[d]) {
            PlaneLinel linel;
            linel.first[d] = ini[d];
            linel.second = d1;
            for (CellDir
                 k = std::min(ini[d1], end[d1]);
                 k < std::max(ini[d1], end[d1]); k++) {
                linel.first[d1] = k;
                res.insert(linel);
            }
        }
    }
    return res;
}

void Compressor::addConcaveLinels_(const std::set<PlaneSurfel>& surfs,
                                   const std::vector<Contour>& conts,
                                   std::set<PlaneLinel>& lines) {
    std::set<PlanePoint> concavePoints;
    for (std::size_t c = 0; c < conts.size(); c++) {
        for (std::size_t i = 0; i < conts[c].size(); i++) {
            PlanePoint point = conts[c][i];
            std::size_t numSurfAdj = 0;
            std::size_t numLineAdj = 0;
            for (CellDir diffx = -1; diffx < 1; diffx++) {
                for (CellDir diffy = -1; diffy < 1; diffy++) {
                    PlaneSurfel surfel = point;
                    surfel[0] += diffx;
                    surfel[1] += diffy;
                    if (surfs.count(surfel) != 0) {
                        numSurfAdj++;
                    }
                }
            }
            for (Axis d = 0; d < 2; d++) {
                for (CellDir diff = -1; diff < 1; diff++) {
                    PlaneLinel linel = std::make_pair(point, d);
                    linel.first[d] += diff;
                    if (lines.count(linel) != 0) {
                        numLineAdj++;
                    }
                }
            }
            if ((numSurfAdj > 2) && (numLineAdj < 3)) {
                concavePoints.insert(point);
            }
        }
    }
    for (std::set<PlanePoint>::const_iterator
         it = concavePoints.begin(); it != concavePoints.end(); ++it) {
        std::size_t numLineAdj = 0;
        for (Axis d = 0; d < 2; d++) {
            for (CellDir diff = -1; diff < 1; diff++) {
                PlaneLinel linel = std::make_pair(*it, d);
                linel.first[d] += diff;
                if (lines.count(linel) != 0) {
                    numLineAdj++;
                }
            }
        }
        if (numLineAdj > 2) {
            continue;
        }
        for (Axis d = 0; d < 2; d++) {
            Axis d1 = (d + 1) % 2;
            bool found = false;
            for (CellDir diff = -1; diff < 1; diff++) {
                PlaneLinel linel = std::make_pair(*it, d);
                linel.first[d] += diff;
                if (lines.count(linel) == 0) {
                    lines.insert(linel);
                    while (true) {
                        PlaneLinel aux1, aux2;
                        if (diff < 0) {
                            aux1 = aux2 = std::make_pair(linel.first, d1);
                            aux1.first[d1]--;
                            linel.first[d]--;
                        } else {
                            linel.first[d]++;
                            aux1 = aux2 = std::make_pair(linel.first, d1);
                            aux1.first[d1]--;
                        }
                        if ((lines.count(aux1) != 0) ||
                            (lines.count(aux2) != 0)) {
                            break;
                        }
                        lines.insert(linel);
                    }
                    found = true;
                    break;
                }
            }
            if (found) {
                break;
            }
        }
    }
}

}
