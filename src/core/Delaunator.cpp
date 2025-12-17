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

    std::vector<Point> newVertices;
    newVertices.reserve(targetVertices.size());

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

    IndexPointToId res;
    for (std::size_t index = 0; index < targetVertices.size(); ++index) {
        Point point({ rotatedCoordinates[index][X], rotatedCoordinates[index][Y] });
        newVertices.push_back(point);
        
        PointId pointId(index);
        pointsToIds.insert(IndexPointToId::value_type(pointId, originalIds[index]));
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