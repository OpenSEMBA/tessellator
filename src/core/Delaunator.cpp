#include "Delaunator.h"
#include "utils\Geometry.h"

namespace meshlib::core {

Delaunator::Delaunator(const Coordinates& globalCoordinates) : globalCoordinates_(&globalCoordinates) {}

Elements Delaunator::mesh(const Polygons& constrainingPolygons) const {
    try {
        IdSet targetVertices;

        for (auto& polygon : constrainingPolygons) {
            targetVertices.insert(polygon.begin(), polygon.end());
        }

        checkConstraintsArePlanar(targetVertices);

        IndexPointToId pointsToIds;

        Triangulation cdt = buildCDT(pointsToIds, targetVertices, constrainingPolygons);

        cdt.eraseOuterTrianglesAndHoles();

        return convertFromCDT(cdt, pointsToIds);
    }
    catch (std::out_of_range& e) {
        throw std::runtime_error("Coordinate ids are out of range.");
    }
}

void Delaunator::checkConstraintsArePlanar(const IdSet & targetVertices) const {
    Coordinates targetCoordinates;
    targetCoordinates.reserve(targetVertices.size());
    
    for (auto v : targetVertices) {
        targetCoordinates.push_back(globalCoordinates_->at(v));
    }

    if (!utils::Geometry::areCoordinatesCoplanar(targetCoordinates.begin(), targetCoordinates.end())) {
        throw std::runtime_error("Constraining polygons are not planar");
    }
}

Delaunator::Triangulation Delaunator::buildCDT(
    IndexPointToId& pointsToIds,
    const IdSet& targetVertices,
    const Polygons& constrainingPolygons) const {
    Triangulation cdt;

    auto widestAxes = findWidestAxes(targetVertices);

    std::vector<Point> newVertices;
    newVertices.reserve(targetVertices.size());

    for (CoordinateId v : targetVertices) {
        const auto coordinate = globalCoordinates_->at(v);

        Point point({ coordinate[widestAxes.first], coordinate[widestAxes.second] });
        newVertices.push_back(point);

        PointId pointId(newVertices.size() - 1);
        pointsToIds.insert(IndexPointToId::value_type(pointId, v));
    }

    cdt.insertVertices(newVertices);

    std::vector<CDT::Edge> edges;
    edges.reserve(targetVertices.size());

    for (const auto& polygon : constrainingPolygons) {
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            std::set<PointId> edgeSet({
                pointsToIds.right.at(polygon[i]),
                pointsToIds.right.at(polygon[(i + 1) % polygon.size()])
                });

            edges.emplace_back(*edgeSet.begin(), *edgeSet.rbegin());
        }
    }

    cdt.insertEdges(edges);
    
    return cdt;
}

std::pair<Axis, Axis> Delaunator::findWidestAxes(const IdSet& targetVertexes) const {
    auto start = targetVertexes.begin();
    Coordinate lowestValues = globalCoordinates_->at(*start);
    Coordinate highestValues = lowestValues;

    auto vIt = std::next(start);
    for (vIt; vIt != targetVertexes.end(); ++vIt) {
        const Coordinate& coordinate = globalCoordinates_->at(*vIt);

        for (Axis axis = X; axis <= Z; ++axis) {
            if (coordinate[axis] < lowestValues[axis]) {
                lowestValues[axis] = coordinate[axis];
            }
            if (coordinate[axis] > highestValues[axis]) {
                highestValues[axis] = coordinate[axis];
            }
        }
    }

    auto distance = highestValues - lowestValues;
    Axis shortestDistance = X;
    for (Axis axis = Y; axis <= Z; ++axis) {
        if (distance[axis] < distance[shortestDistance]) {
            shortestDistance = axis;
        }
    }
    std::set<Axis> orderedAxes({ (shortestDistance + 1) % 3, (shortestDistance + 2) % 3 });
    Axis first = *orderedAxes.begin();
    Axis second = *std::next(orderedAxes.begin());

    return std::make_pair(first, second);
}

Elements Delaunator::convertFromCDT(const Triangulation& cdt, const IndexPointToId& pointToId) const {
    auto newTriangles = cdt.triangles;

    Elements result;
    result.reserve(newTriangles.size());

    for (const auto& triangle : newTriangles) {
        CoordinateIds vertices;
        vertices.reserve(3);

        for (PointId id : triangle.vertices) {
            vertices.push_back(pointToId.left.at(id));
        }

        result.emplace_back(vertices, Element::Type::Surface);
    }

    return result;
}

}