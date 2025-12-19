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

        PointToId pointsToIds;

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
    PointToId& pointsToIds,
    const IdSet& targetVertices,
    const Polygons& constrainingPolygons) const {
    Triangulation cdt;

    Coordinates rotatedCoordinates;
    std::vector<CoordinateId> originalIds;
    for (auto const& id : targetVertices) {
        rotatedCoordinates.push_back((*globalCoordinates_)[id]);
        originalIds.push_back(id);
    }

    VecD normal({ 0.,0.,0. });

    for (std::size_t i = 0; i + 2 < rotatedCoordinates.size() && normal.norm() == 0.0; ++i) {
        for (std::size_t j = i + 1; j + 1 < rotatedCoordinates.size() && normal.norm() == 0.0; ++j) {
            for (std::size_t k = j + 1; k < rotatedCoordinates.size() && normal.norm() == 0.0; ++k) {
                normal = utils::Geometry::normal(TriV({ rotatedCoordinates[i], rotatedCoordinates[j], rotatedCoordinates[k] }));
            }
        }   
    }
    
    utils::Geometry::rotateToXYPlane(rotatedCoordinates.begin(), rotatedCoordinates.end(), normal);

    
    for (std::size_t index = 0; index < targetVertices.size(); ++index) {
        Point point({ rotatedCoordinates[index][X], rotatedCoordinates[index][Y] });
        
        PointId pointId(index);
        pointsToIds.insert(PointToId::value_type(point, originalIds[index]));
    }

    std::vector<Point> newVertices;
    newVertices.reserve(pointsToIds.left.size());

    for (auto& leftPointIt = pointsToIds.left.begin(); leftPointIt != pointsToIds.left.end(); ++leftPointIt) {
        const Point& point = leftPointIt->first;
        newVertices.push_back(point);
    }

    cdt.insertVertices(newVertices);

    std::vector<CDT::Edge> edges;
    edges.reserve(targetVertices.size());

    for (const auto& polygon : constrainingPolygons) {
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            auto leftCoordIdIt = pointsToIds.right.find(polygon[i]);
            auto leftPointIt = pointsToIds.project_left(leftCoordIdIt);
            PointId leftId = PointId(std::distance(pointsToIds.left.begin(), leftPointIt));

            auto rightCoordIdIt = pointsToIds.right.find(polygon[(i + 1) % polygon.size()]);
            auto rightPointIt = pointsToIds.project_left(rightCoordIdIt);
            PointId rightId = PointId(std::distance(pointsToIds.left.begin(), rightPointIt));

            edges.emplace_back(leftId, rightId);
        }
    }

    cdt.insertEdges(edges);
    
    return cdt;
}

Elements Delaunator::convertFromCDT(const Triangulation& cdt, const PointToId& pointToId) const {
    auto newTriangles = cdt.triangles;

    Elements result;
    result.reserve(newTriangles.size());

    for (const auto& triangle : newTriangles) {
        CoordinateIds vertices;
        vertices.reserve(3);

        for (PointId id : triangle.vertices) {
            const Point& point = cdt.vertices[id];
            vertices.push_back(pointToId.left.at(point));
        }

        result.emplace_back(vertices, Element::Type::Surface);
    }

    return result;
}

}