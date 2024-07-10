#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

namespace meshlib {
namespace cgal {

template <class CoordIt>
class LSFPlane {
public:
	LSFPlane(const CoordIt ini, const CoordIt end);

    VecD getNormal() const;
    bool arePointsInPlane(const CoordIt ini, const CoordIt end) const;
private:
    typedef double                      FT;
    typedef CGAL::Simple_cartesian<FT>  K;

    typedef K::Plane_3                  Plane;
    typedef K::Point_2                  Point2;
    typedef K::Point_3                  Point;
    typedef K::Vector_3					Vector3;
    typedef K::Direction_3			    Direction3;

    Plane plane_;
};

template <class CoordIt>
LSFPlane<CoordIt>::LSFPlane(const CoordIt ini, const CoordIt end)
{

    std::vector<Point> points;
    points.reserve(std::distance(ini, end));
    for (auto it = ini; it != end; ++it) {
        points.push_back(Point((*it)[0], (*it)[1], (*it)[2]));
    }

    linear_least_squares_fitting_3(points.begin(), points.end(), plane_, CGAL::Dimension_tag<0>());
    
}

template <class CoordIt>
VecD LSFPlane<CoordIt>::getNormal() const
{
    Vector3 normal = plane_.orthogonal_vector();
    double norm = sqrt(std::pow(normal.x(), 2) + std::pow(normal.y(), 2) + std::pow(normal.z(), 2));
    return VecD({ normal.x() / norm, normal.y() / norm, normal.z() / norm });
}

template <class CoordIt>
bool LSFPlane<CoordIt>::arePointsInPlane(const CoordIt ini, const CoordIt end) const
{
    std::vector<Point> points;
    points.reserve(std::distance(ini, end));
    for (auto it = ini; it != end; ++it) {
        points.push_back(Point((*it)[0], (*it)[1], (*it)[2]));
    }

    for (const auto& point : points) {
        if (!plane_.has_on(point)) {
            return false;
        }
    }
    return true;
}

}
}
