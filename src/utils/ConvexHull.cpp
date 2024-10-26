#include "ConvexHull.h"
#include "utils/Geometry.h"

#include <numeric>

namespace meshlib::utils {

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

    std::vector<CoordinateId> res;

    return res;
}


}