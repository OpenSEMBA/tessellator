#include "ConvexHull.h"
#include "utils/Geometry.h"

#include <numeric>

namespace meshlib {
namespace cgal {

using namespace utils;

const double ConvexHull::COPLANARITY_ANGLE_TOLERANCE = 0.1;

ConvexHull::ConvexHull(const Coordinates* global)
{
    if (global == nullptr) {
        throw std::runtime_error("Global list of coordinates must be defined");
    }
    globalCoords_ = global;
}

std::vector<CoordinateId> ConvexHull::get(const IdSet& ids) const
{
    assert(ids.size() > 1);

    const IndexPointToId pointToId = buildPointsInIndex(ids);
    
    std::vector<Point_2> points;
    for (auto const& p : pointToId) {
        points.push_back(p.first);
    }

    std::vector<std::size_t> indices(pointToId.size()), out;
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(
        indices.begin(), indices.end(),
        std::back_inserter(out),
        Convex_hull_traits_2(CGAL::make_property_map(points)));

    std::vector<CoordinateId> res;
    for (auto const& c : out) {
        auto const it = pointToId.find(points[c]);
        if (it != pointToId.end()) {
            res.push_back(it->second);
        }
    }

    return res;
}

ConvexHull::IndexPointToId ConvexHull::buildPointsInIndex(
    const IdSet& inIds) const
{
    Coordinates cs;
    cs.reserve(inIds.size());
    std::vector<CoordinateId> originalIds;
    originalIds.reserve(inIds.size());

    for (auto const& id : inIds) {
        cs.push_back((*globalCoords_)[id]);
        originalIds.push_back(id);
    }
    utils::Geometry::rotateToXYPlane(cs.begin(), cs.end());
    
    IndexPointToId res;
    for (std::size_t i = 0; i < cs.size(); i++) {
        res.emplace(Point_2(cs[i](0), cs[i](1)), originalIds[i]);
    }
    return res;
}



}
}