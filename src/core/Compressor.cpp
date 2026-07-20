#include "Compressor.h"

#include <algorithm>
#include <queue>

#include "types/Mesh.h"
#include "utils/GridTools.h"

namespace meshlib::core {

using meshlib::Sign;
using meshlib::SignedAxis;
using meshlib::PlanePoint;
using meshlib::PlaneLinel;
using meshlib::PlaneSurfel;
using meshlib::PlaneSurface;
using meshlib::Contour;
using meshlib::CrossLine;

std::size_t Compressor::compressSurfacesInMesh(Mesh& mesh) {
    std::size_t totalOriginal = 0;
    std::size_t totalCompressed = 0;

    for (Group& group : mesh.groups) {
        std::vector<Element> surfaces;
        
        for (const Element& elem : group.elements) {
            if (elem.type == Element::Type::Surface) {
                surfaces.push_back(elem);
            }
        }

        if (surfaces.empty()) {
            continue;
        }

        totalOriginal += surfaces.size();
        std::vector<Element> compressedSurfaces = compressSurfaces_(mesh.coordinates, surfaces);
        totalCompressed += compressedSurfaces.size();

        // Build new elements vector with compressed surfaces
        std::vector<Element> newElements;
        ElementId surfaceIdx = 0;
        for (ElementId e = 0; e < group.elements.size(); e++) {
            if (group.elements[e].type == Element::Type::Surface) {
                if (surfaceIdx < compressedSurfaces.size()) {
                    newElements.push_back(compressedSurfaces[surfaceIdx]);
                    surfaceIdx++;
                }
            } else {
                newElements.push_back(group.elements[e]);
            }
        }
        group.elements = std::move(newElements);
    }

    return totalOriginal - totalCompressed;
}

std::size_t Compressor::compressLinesInMesh(Mesh& mesh, const std::vector<Element::Type> & dimensionPolicy) {
    std::size_t totalOriginal = 0;
    std::size_t totalCompressed = 0;

    std::vector<bool> compress;
    if (dimensionPolicy.size() == 0){
        compress = std::vector<bool>(mesh.groups.size(), true);
    }
    else{
        compress.reserve(mesh.groups.size());
        for (auto policy: dimensionPolicy){
            compress.emplace_back(policy == Element::Type::Volume || policy == Element::Type::Surface);
        }
    }

    for (GroupId g = 0; g < mesh.groups.size(); ++g){
        if (!compress[g]){
            continue;
        }

        std::vector<Element> lines;
        
        for (const Element& elem : mesh.groups[g].elements) {
            if (elem.type == Element::Type::Line) {
                lines.push_back(elem);
            }
        }

        if (lines.empty()) {
            continue;
        }

        totalOriginal += lines.size();
        std::vector<Element> compressedLines = compressLines_(mesh.coordinates, lines);
        totalCompressed += compressedLines.size();

        // Build new elements vector with compressed lines
        std::vector<Element> newElements;
        ElementId lineIdx = 0;
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            if (mesh.groups[g].elements[e].type == Element::Type::Line) {
                if (lineIdx < compressedLines.size()) {
                    newElements.push_back(compressedLines[lineIdx]);
                    lineIdx++;
                }
            } else {
                newElements.push_back(mesh.groups[g].elements[e]);
            }
        }
        mesh.groups[g].elements = std::move(newElements);
    }

    return totalOriginal - totalCompressed;
}

std::vector<Element> Compressor::compressLines_(
        const std::vector<Relative>& coords,
        const std::vector<Element>& lines) {
    std::vector<Element> result;
    std::map<std::pair<GridLine, SignedAxis>,
              std::vector<ElementId>> signDirLines;
    for (std::size_t l = 0; l < lines.size(); l++) {
        std::array<Cell, 2> auxCells;
        auxCells[0] = utils::GridTools::toCell(coords[lines[l].vertices[0]]);
        auxCells[1] = utils::GridTools::toCell(coords[lines[l].vertices[1]]);
        GridLine gridLine;
        Sign sign = 1;
        Axis direction = 0;
        for (Axis d = X; X <= Z; d++) {
            if (auxCells[0](d) != auxCells[1](d)) {
                if (auxCells[0](d) > auxCells[1](d)) {
                    sign = -1;
                }
                direction = d;
                Axis d1 = (d + 1) % 3;
                Axis d2 = (d + 2) % 3;
                gridLine[0] = auxCells[0](d1);
                gridLine[1] = auxCells[0](d2);
                break;
            }
        }
        signDirLines[std::make_pair(gridLine,
                                     std::make_pair(sign, direction))].push_back(l);
    }
    for (std::map<std::pair<GridLine, SignedAxis>,
                   std::vector<ElementId>>::const_iterator
          it = signDirLines.begin(); it != signDirLines.end(); ++it) {
        std::vector<Element> auxElems;
        for (std::size_t i = 0; i < it->second.size(); i++) {
            auxElems.push_back(lines[it->second[i]]);
        }
        std::vector<Element> compressedLines =
            compressDirSignLines_(coords,
                                   it->first.second,
                                   auxElems);
        result.insert(result.end(), compressedLines.begin(), compressedLines.end());
    }
    return result;
}

std::vector<Element> Compressor::compressDirSignLines_(
        const std::vector<Relative>& coords,
        const SignedAxis& signedDir,
        const std::vector<Element>& lines) {
    std::vector<Element> result;
    std::map<RelativeId, std::set<ElementId>> relativeLines;
    std::map<ElementId, std::set<RelativeId>> lineRelatives;
    for (std::size_t l = 0; l < lines.size(); l++) {
        for (RelativeId vertex : lines[l].vertices) {
            relativeLines[vertex].insert(l);
            lineRelatives[l].insert(vertex);
        }
    }
    std::set<ElementId> visitedLineIds;
    for (std::map<ElementId, std::set<RelativeId>>::const_iterator
         itExt = lineRelatives.begin(); itExt != lineRelatives.end(); ++itExt) {
        if (visitedLineIds.count(itExt->first) == 0) {
            RelativeId minCell = lines[itExt->first].vertices[0];
            RelativeId maxCell = lines[itExt->first].vertices[1];
            std::queue<ElementId> linesToVisit;
            linesToVisit.push(itExt->first);
            visitedLineIds.insert(itExt->first);
            while (!linesToVisit.empty()) {
                ElementId elem = linesToVisit.front();
                linesToVisit.pop();
                for (std::size_t i = 0; i < 2; i++) {
                    if (coords[minCell] > coords[lines[elem].vertices[i]]) {
                        minCell = lines[elem].vertices[i];
                    }
                    if (coords[maxCell] < coords[lines[elem].vertices[i]]) {
                        maxCell = lines[elem].vertices[i];
                    }
                }
                for (std::set<RelativeId>::const_iterator
                     itCell  = lineRelatives[elem].begin();
                     itCell != lineRelatives[elem].end(); ++itCell) {
                    for (std::set<ElementId>::const_iterator
                         itLine  = relativeLines[*itCell].begin();
                         itLine != relativeLines[*itCell].end(); ++itLine) {
                        if (visitedLineIds.count(*itLine) == 0) {
                            linesToVisit.push(*itLine);
                            visitedLineIds.insert(*itLine);
                        }
                    }
                }
            }
            Element newElem;
            newElem.type = Element::Type::Line;
            newElem.vertices.push_back(minCell);
            newElem.vertices.push_back(maxCell);
            if (signedDir.first < 0) {
                std::swap(newElem.vertices[0], newElem.vertices[1]);
            }
            result.push_back(newElem);
        }
    }
    return result;
}

std::vector<Element> Compressor::compressSurfaces_(
        std::vector<Relative>& coords,
        const std::vector<Element>& surfaces) {
            
    std::vector<Element> result;
    std::map<std::pair<CellDir, SignedAxis>, std::vector<ElementId>> signDirSurfs;

    for (std::size_t s = 0; s < surfaces.size(); s++) {
        if (surfaces[s].vertices.size() != 4) {
            result.push_back(surfaces[s]);
            continue;
        }
        std::array<Cell, 3> auxCells;
        auxCells[0] = utils::GridTools::toCell(coords[surfaces[s].vertices[0]]);
        auxCells[1] = utils::GridTools::toCell(coords[surfaces[s].vertices[1]]);
        auxCells[2] = utils::GridTools::toCell(coords[surfaces[s].vertices[2]]);
        CellDir gridSurface;
        Sign sign = 1;
        Axis direction = 0;
        for (Axis d = X; d <= Z; d++) {
            if (auxCells[0](d) == auxCells[2](d)) {
                Cell normal = (auxCells[1] - auxCells[0]) ^
                              (auxCells[2] - auxCells[0]);
                if (normal(d) >= 0) {
                    sign = 1;
                } else {
                    sign = -1;
                }
                direction = d;
                gridSurface = auxCells[0](d);
                break;
            }
        }
        signDirSurfs[std::make_pair(gridSurface,
                                    std::make_pair(sign, direction))].push_back(s);
    }
    for (std::map<std::pair<CellDir, SignedAxis>,
                  std::vector<ElementId>>::const_iterator
         it = signDirSurfs.begin(); it != signDirSurfs.end(); ++it) {
        std::vector<Element> auxElems;
        for (std::size_t i = 0; i < it->second.size(); i++) {
            auxElems.push_back(surfaces[it->second[i]]);
        }
        std::vector<Element> compressedSurfaces =
            compressSurfacesWithSameNormal_(coords, it->first.second, auxElems);
        result.insert(result.end(), compressedSurfaces.begin(), compressedSurfaces.end());
    }
    return result;
}

std::vector<Element> Compressor::compressSurfacesWithSameNormal_(
        std::vector<Relative>& coords,
        const SignedAxis& signedDir,
        const std::vector<Element>& surfaces) {
    std::vector<Element> result;
    std::map<LinIds, std::set<ElementId>> edgeSurfaces;
    std::map<ElementId, std::set<LinIds>> surfaceEdges;
    for (ElementId s = 0; s < surfaces.size(); s++) {
        for (std::size_t i = 0; i < 4; i++) {
            std::size_t j = (i + 1) % 4;
            LinIds edge;
            edge[0] = surfaces[s].vertices[i];
            edge[1] = surfaces[s].vertices[j];
            std::sort(edge.begin(), edge.end());
            edgeSurfaces[edge].insert(s);
            surfaceEdges[s].insert(edge);
        }
    }
    std::set<ElementId> visitedSurfaceIds;
    for (std::map<ElementId, std::set<LinIds>>::const_iterator
         itExt = surfaceEdges.begin(); itExt != surfaceEdges.end(); ++itExt) {
        if (visitedSurfaceIds.count(itExt->first) == 0) {
            std::set<ElementId> connectedSurfaceIds;
            std::queue<ElementId> surfacesToVisit;
            surfacesToVisit.push(itExt->first);
            visitedSurfaceIds.insert(itExt->first);
            while (!surfacesToVisit.empty()) {
                ElementId e = surfacesToVisit.front();
                surfacesToVisit.pop();
                connectedSurfaceIds.insert(e);
                for (std::set<LinIds>::const_iterator
                     itLine  = surfaceEdges[e].begin();
                     itLine != surfaceEdges[e].end(); ++itLine) {
                    for (std::set<ElementId>::const_iterator
                         itSurf  = edgeSurfaces[*itLine].begin();
                         itSurf != edgeSurfaces[*itLine].end(); ++itSurf) {
                        if (visitedSurfaceIds.count(*itSurf) == 0) {
                            surfacesToVisit.push(*itSurf);
                            visitedSurfaceIds.insert(*itSurf);
                        }
                    }
                }
            }
            std::vector<Element> connectedSurfaces;
            for (std::set<ElementId>::const_iterator
                 it = connectedSurfaceIds.begin(); it != connectedSurfaceIds.end(); ++it) {
                connectedSurfaces.push_back(surfaces[*it]);
            }
            std::vector<Element> compressedSurfaces = compressConnectedSurfaces_(coords, signedDir, connectedSurfaces);
            result.insert(result.end(), compressedSurfaces.begin(), compressedSurfaces.end());
        }
    }
    return result;
}

std::vector<Element> Compressor::compressConnectedSurfaces_(
        std::vector<Relative>& coords,
        const SignedAxis& signDir,
        const std::vector<Element>& surfs) {
    std::vector<Element> result;
    Axis d = signDir.second;
    Axis d1 = (d + 1) % 3;
    Axis d2 = (d + 2) % 3;
    CellDir plane = utils::GridTools::toCell(coords[surfs[0].vertices[0]])(d);
    std::set<PlaneSurfel> surfels;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        std::pair<PlanePoint, PlanePoint> fullSurfacePoints;
        fullSurfacePoints.first[0] =
            utils::GridTools::toCell(coords[surfs[s].vertices[0]])(d1);
        fullSurfacePoints.first[1] =
            utils::GridTools::toCell(coords[surfs[s].vertices[0]])(d2);
        fullSurfacePoints.second[0] =
            utils::GridTools::toCell(coords[surfs[s].vertices[2]])(d1);
        fullSurfacePoints.second[1] =
            utils::GridTools::toCell(coords[surfs[s].vertices[2]])(d2);
        CellDir i0 = std::min(fullSurfacePoints.first[0], fullSurfacePoints.second[0]);
        CellDir i1 = std::max(fullSurfacePoints.first[0], fullSurfacePoints.second[0]);
        CellDir j0 = std::min(fullSurfacePoints.first[1], fullSurfacePoints.second[1]);
        CellDir j1 = std::max(fullSurfacePoints.first[1], fullSurfacePoints.second[1]);
        for (CellDir i = i0; i < i1; i++) {
            for (CellDir j = j0; j < j1; j++) {
                PlaneSurfel surfel = {{i, j}};
                surfels.insert(surfel);
            }
        }
    }
    std::vector<PlaneSurface> maximalRectangles = compressSurfelsIntoMaximalRectangles_(surfels);
    CoordinateMap coordMap;
    for (std::size_t s = 0; s < surfs.size(); s++) {
        for (std::size_t i = 0; i < 4; i++) {
            RelativeId coordId = surfs[s].vertices[i];
            Relative   coord   = coords[coordId];
            coordMap[coord] = coordId;
        }
    }
    for (std::size_t e = 0; e < maximalRectangles.size(); e++) {
        std::array<Cell, 4> corners;
        corners[0](d) = corners[2](d) = plane;
        corners[0](d1) = maximalRectangles[e].first[0];
        corners[0](d2) = maximalRectangles[e].first[1];
        corners[2](d1) = maximalRectangles[e].second[0];
        corners[2](d2) = maximalRectangles[e].second[1];
        corners[1] = corners[3] = corners[0];
        corners[1](d1) = corners[2](d1);
        corners[3](d2) = corners[2](d2);
        if (signDir.first < 0) {
            std::swap(corners[1], corners[3]);
        }
        Element newSurface;
        newSurface.type = Element::Type::Surface;
        for (std::size_t i = 0; i < 4; i++) {
            Relative rel = utils::GridTools::toRelative(corners[i]);
            if (coordMap.count(rel) == 0) {
                coordMap[rel] = coords.size();
                coords.push_back(rel);
            }
            newSurface.vertices.push_back(coordMap[rel]);
        }
        result.push_back(newSurface);
    }
    return result;
}

std::vector<PlaneSurface> Compressor::compressSurfelsIntoMaximalRectangles_(
        const std::set<PlaneSurfel>& surfels) {
    std::vector<PlaneSurface> result;
    const std::vector<Contour>& contours = getContours_(surfels);
    std::array<std::vector<CrossLine>, 2> crossingLines = getCrossingLines_(surfels, contours);
    crossingLines = getMaxCompactedLines_(crossingLines);
    std::set<PlaneLinel> linels;
    for (std::size_t c = 0; c < contours.size(); c++) {
        for (std::size_t i = 0; i < contours[c].size(); i++) {
            std::size_t j = (i + 1) % contours[c].size();
            std::set<PlaneLinel> aux = getLinelsBetween_(contours[c][i], contours[c][j]);
            linels.insert(aux.begin(), aux.end());
        }
    }
    for (Axis d = 0; d < 2; d++) {
        Axis d1 = (d + 1) % 2;
        for (std::size_t i = 0; i < crossingLines[d].size(); i++) {
            PlanePoint ini, end;
            ini[d1] = end[d1] = crossingLines[d][i].first;
            ini[d] = crossingLines[d][i].second.first;
            end[d] = crossingLines[d][i].second.second;
            std::set<PlaneLinel> aux = getLinelsBetween_(ini, end);
            linels.insert(aux.begin(), aux.end());
        }
    }
    addConcaveLinels_(surfels, contours, linels);
    std::map<PlaneLinel, std::set<PlaneSurfel>> edgeSurfels;
    std::map<PlaneSurfel, std::set<PlaneLinel>> surfelEdges;
    for (std::set<PlaneSurfel>::const_iterator
         it = surfels.begin(); it != surfels.end(); ++it) {
        surfelEdges.insert(std::make_pair(*it, std::set<PlaneLinel>()));
        for (Axis d = 0; d < 2; d++) {
            for (CellDir diff = -1; diff <= 1; diff += 2) {
                PlaneLinel linel = getSurfaceEdge_(*it, diff, d);
                if (linels.count(linel) == 0) {
                    surfelEdges[*it].insert(linel);
                    edgeSurfels[linel].insert(*it);
                }
            }
        }
    }
    std::set<PlaneSurfel> visitedSurfels;
    for (std::map<PlaneSurfel, std::set<PlaneLinel>>::const_iterator
         itSurfExt = surfelEdges.begin();
         itSurfExt != surfelEdges.end(); ++itSurfExt) {
        if (visitedSurfels.count(itSurfExt->first) == 0) {
            std::queue<PlaneSurfel> surfelsToVisit;
            surfelsToVisit.push(itSurfExt->first);
            visitedSurfels.insert(itSurfExt->first);
            PlanePoint minPoint = itSurfExt->first;
            PlanePoint maxPoint = itSurfExt->first;
            while (!surfelsToVisit.empty()) {
                PlaneSurfel surfel = surfelsToVisit.front();
                surfelsToVisit.pop();
                if (surfel < minPoint) {
                    minPoint = surfel;
                }
                if (surfel > maxPoint) {
                    maxPoint = surfel;
                }
                for (std::set<PlaneLinel>::const_iterator
                     itLin = surfelEdges[surfel].begin();
                     itLin != surfelEdges[surfel].end(); ++itLin) {
                    for (std::set<PlaneSurfel>::const_iterator
                         itSurfInt = edgeSurfels[*itLin].begin();
                         itSurfInt != edgeSurfels[*itLin].end(); ++itSurfInt) {
                        if (visitedSurfels.count(*itSurfInt) == 0) {
                            surfelsToVisit.push(*itSurfInt);
                            visitedSurfels.insert(*itSurfInt);
                        }
                    }
                }
            }
            maxPoint[0]++;
            maxPoint[1]++;
            result.push_back(std::make_pair(minPoint, maxPoint));
        }
    }
    return result;
}

std::vector<Contour> Compressor::getContours_(const std::set<PlaneSurfel>& surfels) {
    std::vector<Contour> result;
    if (surfels.empty()) {
        return result;
    }
    std::set<PlaneLinel> visitedEdges;
    result.push_back(getContourFromStartingEdge_(
        getSurfaceEdge_(*surfels.begin(), -1, 0),
        surfels,
        visitedEdges
    ));
    for (std::set<PlaneSurfel>::const_iterator
         it = surfels.begin(); it != surfels.end(); ++it) {
        for (Axis d = 0; d < 2; d++) {
            for (CellDir diff = -1; diff <= 1; diff += 2) {
                PlaneSurfel adjSurf;
                PlaneLinel adjEdge;
                adjSurf = *it;
                adjSurf[d] += diff;
                adjEdge = getSurfaceEdge_(*it, diff, d);
                if ((surfels.find(adjSurf) == surfels.end()) &&
                    (visitedEdges.find(adjEdge) == visitedEdges.end())) {
                    result.push_back(getContourFromStartingEdge_(adjEdge, surfels, visitedEdges));
                }
            }
        }
    }
    return result;
}

Contour Compressor::getContourFromStartingEdge_(
        const PlaneLinel& from,
        const std::set<PlaneSurfel>& surfs,
        std::set<PlaneLinel>& visitedEdges) {
    Contour result;
    std::queue<PlaneLinel> q;
    if (visitedEdges.find(from) != visitedEdges.end()) {
        return result;
    }
    std::vector<PlaneLinel> lines;
    q.push(from);
    visitedEdges.insert(from);
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
                if (visitedEdges.find(adjEdge) == visitedEdges.end()) {
                    q.push(adjEdge);
                    visitedEdges.insert(adjEdge);
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
                if (visitedEdges.find(adjEdge) == visitedEdges.end()) {
                    q.push(adjEdge);
                    visitedEdges.insert(adjEdge);
                    lines.push_back(adjEdge);
                    break;
                }
                continue;
            } else {
                PlaneLinel adjEdge = getSurfaceEdge_(adjSurf1, -diff, d0);
                if (visitedEdges.find(adjEdge) == visitedEdges.end()) {
                    q.push(adjEdge);
                    visitedEdges.insert(adjEdge);
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
                result.push_back(extremes[p / 2]);
                break;
            }
        }
    }
    return result;
}

PlaneLinel Compressor::getSurfaceEdge_(const PlaneSurfel& surfel, const CellDir& diff, const Axis& dir) {
    PlaneLinel result;
    result.first  = surfel;
    result.second = (dir + 1) % 2;
    if (diff > 0) {
        result.first[dir]++;
    }
    return result;
}

std::array<std::vector<CrossLine>, 2>
    Compressor::getCrossingLines_(
        const std::set<PlaneSurfel>& surfs,
        const std::vector<Contour>& conts) {
    std::array<std::vector<CrossLine>, 2> result;
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
                    result[d].push_back(
                        std::make_pair(itMap->first,
                                       std::make_pair(*itSet, *itSetPlus)));
                }
            }
        }
    }
    return result;
}

std::array<std::vector<CrossLine>, 2>
    Compressor::getMaxCompactedLines_(
        const std::array<std::vector<CrossLine>, 2>& cross) {
    std::array<std::vector<CrossLine>, 2> result;
    if (cross[0].size() > cross[1].size()) {
        result[0] = cross[0];
    } else {
        result[1] = cross[1];
    }
    return result;
}

std::set<PlaneLinel> Compressor::getLinelsBetween_(
        const PlanePoint& ini,
        const PlanePoint& end) {
    std::set<PlaneLinel> result;
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
                result.insert(linel);
            }
        }
    }
    return result;
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
