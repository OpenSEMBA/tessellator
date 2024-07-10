#pragma once

#include <array>
#include <vector>
#include <algorithm>

#include "cgal/LSFPlane.h"
#include "Types.h"

namespace meshlib {
namespace utils {

class Geometry {
public:
    static constexpr double NORM_TOLERANCE = 1e-13;

    Geometry() = delete;
    Geometry(const Geometry&) = delete;

    static std::vector<ElementsView> buildDisjointSmoothSets(
        const ElementsView& elems,
        const Coordinates& coords,
        const double smoothingAngle);

    static bool areAdjacentWithSameTopologicalOrientation( const Element&, const Element&);
    static bool areAdjacentLines(const Element&, const Element&);
    

    static TriV asTriV(const Element&, const Coordinates&);
    
    static bool approxEqualLines(const LinV& a, const LinV& b);
    static bool approximatelyAligned(const TriV& a, const TriV& b, const double& angle);
    static bool approximatelyOrientedAligned(const TriV& a, const TriV& b, const double& angle);
    static VecD normal(const TriV& a);
    static VecD getCentroid(const Element&, const std::vector<Coordinate>&);
    static VecD getCentroid(const TriV&);
    static double area(const TriV& tri);
    static bool isDegenerate(const TriV& tri, const double& areaTolerance = NORM_TOLERANCE);
    static bool areCollinear(const Coordinates&);
    template <std::size_t N>
    static std::array<CoordinateId, N> toArray(
            const std::vector<CoordinateId>& ids) {
        std::array<CoordinateId, N> res;
        for (std::size_t i = 0; i < N; i++) {
            res[i] = ids[i];
        }
        return res;
    }

    static VecD getNormal(
        const Coordinates&, 
        double coplanarityAngleTolerance);
    static VecD getMeanNormalOfElements(
        const ElementsView& elements,
        const Coordinates& coords);

    template <class CoordinatesIt>
    static bool areCoordinatesCoplanar(
        const CoordinatesIt ini, 
        const CoordinatesIt end)
    {
        if (std::distance(ini, end) < 3) {
            return false;
        }
        
        Coordinates seed;
        {
            auto it = ini;
            while (isDegenerate(TriV{ *it, *std::next(it), *std::next(it,2) })) {
                ++it;
                if (std::next(it,2) == end) {
                    return false;
                }
            }
            seed = { *it, *std::next(it), *std::next(it,2) };
        }
        
        return cgal::LSFPlane(seed.begin(), seed.end()).arePointsInPlane(ini, end);
    }

    template <class coordinatesIt>
    static void rotateToXYPlane(coordinatesIt ini, coordinatesIt end, VecD normal = VecD({ 0.,0.,0. }))
    {
        if (normal.norm() == 0) {
            normal = cgal::LSFPlane(ini, end).getNormal();
        }

        const VecD z({ 0.0, 0.0, 1.0 });
        const VecD u = normal ^ z;
        const double cTh = normal(2);
        const double sTh = sin(acos(cTh));
        for (auto it = ini; it != end; ++it) {
            const VecD& v = *it;
            (*it)(0) = (cTh + pow(u(0), 2) * (1 - cTh)) * v(0) +
                (u(0) * u(1) * (1 - cTh) - u(2) * sTh) * v(1) +
                (u(0) * u(2) * (1 - cTh) + u(1) * sTh) * v(2);
            (*it)(1) = (u(1) * u(0) * (1 - cTh) + u(2) * sTh) * v(0) +
                (cTh + pow(u(1), 2) * (1 - cTh)) * v(1) +
                (u(1) * u(2) * (1 - cTh) - u(0) * sTh) * v(2);
            (*it)(2) = 0.0;
        }
    }
    
    static Coordinates projectToPlane(
        const Coordinates& toProject,
        const VecD& normal);

};

}
}

