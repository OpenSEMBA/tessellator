#include "Smoother.h"

#include "utils/CoordGraph.h"
#include "utils/ElemGraph.h"
#include "utils/Cleaner.h"
#include "utils/Geometry.h"
#include "utils/Tools.h"

#include "cgal/Delaunator.h"

#include <algorithm>
#include <stdexcept>
#include <assert.h>
#include <iostream>

namespace meshlib {
namespace tessellator {

using namespace utils;

SmootherTools::SmootherTools(const Grid& grid) :
    GridTools(grid)
{}

CoordinateId SmootherTools::getClosestEndOfPaths(
    const std::vector<CoordGraph::Path>& paths)
{
    if (paths.size() == 0) {
        throw std::runtime_error("At least one path is needed.");
    }
    for (auto const& path : paths) {
        if (path.size() == 0) {
            throw std::runtime_error("Size zero paths are not allowed.");
        }
    }

    std::size_t smallest = paths.begin()->size();
    CoordinateId res = paths.begin()->back();
    for (auto const& path : paths) {
        if (path.size() < smallest) {
            smallest = path.size();
            res = path.back();
        }
    }

    return res;
}

void SmootherTools::updateCoordinates(
    Coordinates& cs, std::map<CoordinateId, Coordinate> toMove)
{
    if (toMove.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(writingCoordinates_);
    for (auto it : toMove) {
        cs[it.first] = it.second;
    }
}

void SmootherTools::collapsePointsOnFeatureEdges(
    Coordinates& coords,
    const ElementsView& patch,
    const SingularIds& singularIds)
{
    CoordGraph pt = CoordGraph(patch);
    CoordGraph edges = pt.getBoundaryGraph().intersect(singularIds.featureIds());
    if (edges.verticesSize() == 0) {
        return;
    }

    IdSet validInterior, validExterior;
    {
        IdSet interior, exterior;
        std::tie(exterior, interior) = classifyIds(
            edges.getVertices(), [&](auto id) {
                return edges.getAdjacentVertices(id).size() != 2;
            });
        if (exterior.empty()) {
            return;
        }

        validInterior = classifyIds(interior, [&](auto id) {
            return
                !singularIds.contourIds().count(id)
                && !singularIds.cornerIds().count(id);
            }).first;

        if (validInterior.empty()) {
            return;
        }

        {
            IdSet inCorners = intersectWithIdSet(interior, singularIds.cornerIds());
            IdSet inContour = intersectWithIdSet(interior, singularIds.contourIds());
            validExterior = mergeIds(exterior, mergeIds(inCorners, inContour));
        }
    }

    std::map<CoordinateId, Coordinate> toMove;
    for (auto const& i : validInterior) {
        if (isRelativeOnCellCorner(coords[i])) {
            continue;
        }

        Coordinate closest = closestByDistance(coords, i, pt.getClosestVerticesInSet(i, validExterior));
        if (isRelativeOnCellFace(coords[i]) && !areCoordOnSameFace(coords[i], closest)) {
            continue;
        }
        if (isRelativeOnCellEdge(coords[i]) && !areCoordOnSameEdge(coords[i], closest)) {
            continue;
        }
        toMove[i] = closest;
    }

    updateCoordinates(coords, toMove);
}

Coordinate SmootherTools::closestByDistance(
    const Coordinates& coord,
    const CoordinateId& id,
    const IdSet& candidates)
{
    assert(!candidates.empty());

    double minDist = std::numeric_limits<double>::max();
    CoordinateId closestId = std::numeric_limits<CoordinateId>::max();
    for (auto const& c: candidates) {
        double dist = (coord[id] - coord[c]).norm();
        if (dist < minDist) {
            minDist = dist;
            closestId = c;
        }
    }
    return coord[closestId];
}

SmootherTools::SingularIds SmootherTools::buildSingularIds(
    const Elements& elems,
    const Coordinates& coords,
    double smoothSetAngle) const
{
    IdSet featureIds, contourIds, cornerIds;
    
    contourIds = CoordGraph(elems).getBoundaryGraph().getVertices();
    
    for (auto const& c : buildCellElemMap(elems, coords)) {
        const std::vector<CoordGraph> graphs = CoordGraph::buildFromElementsViews(
            Geometry::buildDisjointSmoothSets(c.second, coords, smoothSetAngle));

        for (auto const& g : graphs) {
            const std::size_t i = &g - &graphs.front();
            for (std::size_t j = i + 1; j < graphs.size(); j++) {
                auto edge = g.intersect(graphs[j]).getVertices();
                for (auto const id : edge) {
                    if (featureIds.count(id)) {
                        cornerIds.insert(id);
                    }
                }
                featureIds.insert(edge.begin(), edge.end());
            }
        }
    }

    return SingularIds(featureIds, contourIds, cornerIds);
}


void SmootherTools::collapsePointsOnCellEdges(
    Coordinates& coords,
    const ElementsView& patch,
    const SingularIds& singularIds,
    double alignmentAngle)
{
    {
        IdSet vertices = CoordGraph(patch).getVertices();

        if (!std::all_of(
            vertices.begin(), vertices.end(),
            [&](const CoordinateId& cId) {
                return
                    isRelativeOnCellFace(coords[cId]) ||
                    isRelativeOnCellEdge(coords[cId]) ||
                    isRelativeOnCellCorner(coords[cId]);
            }
        )) {
            return;
        }
    }

    Elements lines = buildBoundaryContourAsLines(coords, patch);
    
    auto const & protectedIds = singularIds.edgeIds();
    for (auto const& aEG : ElemGraph(lines, coords).splitByWeight(alignmentAngle)) {
        CoordGraph cG(aEG.getAsElements(lines));
        IdSet interior = cG.getInterior();
        if (interior.empty()) {
            continue;
        }

        IdSet interiorValid = intersectWithIdSet(interior, protectedIds);
        
        IdSet movable = classifyIds(interior, [&](auto i) {return !protectedIds.count(i); }).first;
        IdSet validIds = mergeIds(cG.getExterior(), interiorValid);
        
        std::map<CoordinateId, Coordinate> toMove;
        for (auto const& i : movable) {
            if (isRelativeOnCellCorner(coords[i])) {
                continue;
            }
            Coordinate closest = closestByDistance(coords, i, cG.getClosestVerticesInSet(i, validIds));
            if (isRelativeOnCellFace(coords[i]) && !areCoordOnSameFace(coords[i], closest)) {
                continue;
            }
            if (isRelativeOnCellEdge(coords[i]) && !areCoordOnSameEdge(coords[i], closest)) {
                continue;
            }
            toMove[i] = closest;
        }
        updateCoordinates(coords, toMove);
    }

}

IdSet SmootherTools::getClosestValidByDistanceInCycle(
    const CoordinateId& id,
    const CoordGraph::Path& cycle,
    const IdSet& valid)
{
    IdSet candidates;
    auto forwardPath = pathFromIdToAnyTarget(id, cycle, true, valid);
    if (!forwardPath.empty()) {
        candidates.insert(forwardPath.back());
    }
    auto backwardsPath = pathFromIdToAnyTarget(id, cycle, false, valid);
    if (!backwardsPath.empty()) {
        candidates.insert(backwardsPath.back());
    }
    return candidates;
}

void SmootherTools::collapsePointsOnCellFaces(
    Coordinates& coords,
    const ElementsView& patch,
    const SingularIds& sIds)
{
    std::map<CoordinateId, Coordinate> toMove;
    std::map <CoordGraph::Path, std::pair<IdSet, IdSet>> cyclesToValidOrOnFace;
    for (auto const& cycle : CoordGraph(patch).getBoundaryGraph().findCycles()) {
        cyclesToValidOrOnFace.emplace( 
            cycle,
            classifyIds(IdSet(cycle.begin(), cycle.end()),
                [&](const CoordinateId& id) {
                    return !isRelativeOnCellFace(coords[id]) || sIds.edgeIds().count(id) != 0;
                })
        );
    }

    for (auto const& kv : cyclesToValidOrOnFace) {
        const CoordGraph::Path& cycle = kv.first;
        const IdSet& valid = kv.second.first;
        const IdSet& onCellFace = kv.second.second;
        for (auto const& id : onCellFace) {
            try {
                IdSet candidates = getClosestValidByDistanceInCycle(id, cycle, valid);
                if (!candidates.empty()) {
                    Coordinate closest = closestByDistance(coords, id, candidates);
                    if (areCoordOnSameFace(closest, coords[id])) {
                        toMove[id] = closest;
                    }
                }
            }
            catch (...)
            {
                std::stringstream msg;
                msg << "Issue trying to move coord Id " << id 
                    << " with relative coordinates: " << coords[id].str() 
                    << " and absolute coordinates: " << getPos(coords[id]).str() << ". ";
                msg << "Id could not be smoothed.";
                throw std::runtime_error(msg.str());
            }
        }
    }

    updateCoordinates(coords, toMove);
}

bool SmootherTools::patchIsPlanar(
    const Coordinates& cs,
    const ElementsView& patch)
{
    IdSet bound = CoordGraph(patch).getBoundAndInteriorVertices().first;
    Coordinates boundCs;
    boundCs.reserve(bound.size());
    for (const auto& id : bound) {
        boundCs.push_back(cs[id]);
    }

    return Geometry::areCoordinatesCoplanar(boundCs.begin(), boundCs.end());
}

void SmootherTools::remeshBoundary(
    Elements& es,
    Coordinates& cs,
    const Coordinates& meshCs,
    const ElementsView& patch)
{
    CoordGraph g(patch);
    IdSet in = g.getBoundAndInteriorVertices().second;
    if (in.size() < 1) {
        return;
    }

    if (patchIsPlanar(meshCs, patch)){
        remeshWithNoInteriorPoints(es, meshCs, patch);
    }
    else {
        remeshElementsToOneInteriorPoint(es, cs, patch);
    }
}

void SmootherTools::remeshWithNoInteriorPoints(
    Elements& es,
    const Coordinates& cs,
    const ElementsView& patch)
{
    CoordGraph g(patch);
    IdSet in = g.getBoundAndInteriorVertices().second;
    if (in.size() < 1) {
        return;
    }
    CoordinateIds cPolygon = g.getBoundaryGraph().findCycles().front();
    Elements remeshedEls = cgal::Delaunator(&cs).mesh({}, { cPolygon });

    if (hasWrongOrientation(*patch[0], remeshedEls[0], cs)) {
        reorient(remeshedEls);
    }

    for (auto comp = remeshedEls.size(); comp < patch.size(); comp++) {
        remeshedEls.push_back(Element({}, Element::Type::None));
    }
    
    if (remeshedEls.size() != patch.size()) {
        throw std::logic_error("Not all elements have been remeshed");
    }

    const std::lock_guard<std::mutex> lock(writingElements_);
    for (auto comp = 0; comp < patch.size(); comp++) {
        ElementId eId = patch[comp] - &es.front();
        es[eId] = remeshedEls[comp];
    }

}

void SmootherTools::remeshElementsToOneInteriorPoint(
    Elements& es,
    Coordinates& cs,
    const ElementsView& patch)
{
    CoordGraph g(patch);
    IdSet bound, in;
    std::tie(bound, in) = g.getBoundAndInteriorVertices();
    
    if (in.size() < 1) {
        return;
    }

    const CoordinateId uniqueId = *in.begin();
    {
        std::lock_guard<std::mutex> lock(writingCoordinates_);
        for (auto it = ++in.begin(); it != in.end(); it++) {
            const CoordinateId id = *it;
            cs[id] = cs[uniqueId];
        }
    }
    Elements remeshedElements;

    for (Element line : CoordGraph(patch).getBoundaryGraph().getEdgesAsLines()) {
        line.type = Element::Type::Surface;
        line.vertices.push_back(uniqueId);
        remeshedElements.push_back(line);
    }

    if (hasWrongOrientation(*patch[0], remeshedElements[0], cs)) {
        reorient(remeshedElements);
    }
    for (std::size_t i = remeshedElements.size(); i < patch.size(); i++) {
        remeshedElements.push_back(Element({}, Element::Type::None));
    }

    if (remeshedElements.size() != patch.size()) {
        throw std::logic_error("Not all elements have been remeshed");
    }
    assert(remeshedElements.size() == patch.size());
    {
        std::lock_guard<std::mutex> lock(writingElements_);
        for (std::size_t comp = 0; comp < patch.size(); comp++) {
            ElementId eId = patch[comp] - &es.front();
            es[eId] = remeshedElements[comp];
        }
    }

}

CoordGraph::Path SmootherTools::pathFromIdToAnyTarget(
    const CoordinateId& startId,
    const CoordGraph::Path& cycle,
    bool forward,
    const IdSet& target)
{
    auto it = std::find(cycle.begin(), cycle.end(), startId);
    if (it == cycle.end()) {
        throw std::runtime_error("Id does not belong to this cycle.");
    }
    const std::size_t i = it - cycle.begin();

    CoordGraph::Path res(startId);
    for (std::size_t d = 1; d < cycle.size(); d++) {
        CoordinateId idTest;
        if (forward) {
            idTest = cycle[(i + d) % cycle.size()];
        }
        else {
            idTest = cycle[(i + cycle.size() - d) % cycle.size()];
        }
        res.push_back(idTest);

        if (target.count(idTest) != 0) {
            return res;
        }
    }
    return CoordGraph::Path();
}

Coordinates SmootherTools::collapsePointsOnContour(
    const Elements& elems,
    const Coordinates& coords,
    const double alignmentThresholdAngle)
{
    Coordinates res{ coords };
    auto contourIds{ CoordGraph{ elems }.getBoundaryGraph().getVertices() };
    
    for (auto const& c : buildCellElemMap(elems, coords)) {
        Elements lines = CoordGraph(c.second)
            .getBoundaryGraph()
            .intersect(contourIds)
            .getEdgesAsLines();

        for (auto const& aEG : ElemGraph(lines, coords).splitByWeight(alignmentThresholdAngle)) {
            CoordGraph cG(aEG.getAsElements(lines));
            IdSet validContourIds;
            try {
                validContourIds = cG.getExterior();
            }
            catch (...) {
                std::stringstream msg;
                msg << "Issue trying to find exterior during contour smoothing.";
                std::cerr << msg.str() << std::endl;
                throw std::runtime_error(msg.str());
            }

            if (!validContourIds.empty()) {
                for (auto const& interiorId : cG.getInterior()) {
                    res[interiorId] = coords[*validContourIds.begin()];
                }
            }
            
            
        }
    }
    return res;
}

void SmootherTools::collapseInteriorPointsToBound(
    Coordinates& coords,
    const ElementsView& patch)
{
    IdSet bound, interior;
    std::tie(bound, interior) = CoordGraph(patch).getBoundAndInteriorVertices();

    std::map<CoordinateId, Coordinate> toMove;
    for (auto const& vI : interior) {
        toMove[vI] = closestByDistance(coords, vI, bound);
    }

    updateCoordinates(coords, toMove);
}

void SmootherTools::collapseElementsInPatch(
    Elements& es,
    const ElementsView& p,
    const CoordinateId& id,
    const CoordinateId& newId)
{
    for (auto const& e : p) {
        const ElementId eId = e - &es.front();
        for (auto& vId : es[eId].vertices) {
            if (vId == id) {
                vId = newId;
            }
        }
        if (IdSet(es[eId].vertices.begin(), es[eId].vertices.end()).size() < 3) {
            es[eId].vertices.clear();
            es[eId].type = Element::Type::None;
        }
    }
}

Elements SmootherTools::buildBoundaryContourAsLines(
    const Coordinates& coords,
    const ElementsView& patch) const
{
    CoordGraph g = CoordGraph(patch);
    IdSet vertices = g.getVertices();

    IdSet contour = classifyIds(vertices, [&](auto i) {
        return isRelativeOnCellCorner(coords[i])
            || isRelativeOnCellEdge(coords[i]);
        }).first;

    Elements lines;
    Elements allLines = g.getBoundaryGraph().intersect(contour).getEdgesAsLines();
    std::copy_if(
        allLines.begin(), allLines.end(),
        std::back_inserter(lines),
        [&](auto const& e) {return coords[e.vertices[0]] != coords[e.vertices[1]]; });

    return lines;
}

bool SmootherTools::hasWrongOrientation(
    const Element& ref,
    const Element& check,
    const Coordinates& coords)
{
    VecD refNormal = Geometry::normal(Geometry::asTriV(ref, coords));
    VecD checkNormal = Geometry::normal(Geometry::asTriV(check, coords));
    if (checkNormal * refNormal < 0) {
        return true;
    }
    else {
        return false;
    }
}

void SmootherTools::reorient(
    Elements& es)
{
    for (auto& e : es) {
        auto c = e.vertices[0];
        e.vertices[0] = e.vertices[1];
        e.vertices[1] = c;
    }
}
}
}