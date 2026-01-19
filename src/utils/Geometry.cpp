#include "Geometry.h"
#include "utils/CoordGraph.h"
#include "utils/ElemGraph.h"


#include <stdexcept>

namespace meshlib {
namespace utils {

bool Geometry::areAdjacentWithSameTopologicalOrientation(
    const Element& e1,
    const Element& e2)
{
    assert(e1.vertices.size() > 2 && e2.vertices.size() > 2);
    
    for (std::size_t f = 0; f < e2.vertices.size(); f++) {
        std::vector<CoordinateId> vs = { e2.vertices[f], e2.vertices[(f + 1) % e2.vertices.size()] };
        std::reverse(vs.begin(), vs.end());
        auto it = std::search(e1.vertices.begin(), e1.vertices.end(), vs.begin(), vs.end());
        if (it != e1.vertices.end() || 
            (e1.vertices.back() == vs.front() && e1.vertices.front() == vs.back())) {
            return true;
        }
    }
    return false;
}

bool Geometry::areAdjacentLines(
    const Element& e1,
    const Element& e2)
{
    assert(e1.isLine() && e2.isLine());
    
    std::size_t nShared = 0;
    for (CoordinateId const& cId : e1.vertices) {
        if (find(e2.vertices.begin(), e2.vertices.end(), cId) != e2.vertices.end()) {
            nShared++;
        }
        if (nShared == 1 || nShared == 2) {
            return true;
        }
    }
    return false;
}

std::vector<ElementsView> Geometry::buildDisjointSmoothSets(
    const ElementsView& elemsIn,
    const Coordinates& coords,
    const double smoothingAngle)
{
    ElementsView elems;
    std::copy_if(
        elemsIn.begin(), elemsIn.end(),
        std::back_inserter(elems),
        [](const Element* e) { return !e->isNone(); }
    );

    ElemGraph elemGraph(elems, coords);
    std::vector<ElemGraph> smoothGraphs = elemGraph.splitByWeight(smoothingAngle);
    std::vector<ElementsView> smoothSets;
    for (const ElemGraph& smoothGraph : smoothGraphs) {
        ElementsView elemView;
        for (const auto& smoothIds : smoothGraph.getVertices()) {
            elemView.push_back(elems.at(smoothIds));
        }
        smoothSets.push_back(elemView);
    }
    return smoothSets;
}


TriV Geometry::asTriV(const Element& el, const std::vector<Coordinate>& co) {
    if (el.vertices.size() != 3) {
        throw std::logic_error("Invalid conversion from element to TriV");
    }
    TriV res;
    for (std::size_t i = 0; i < el.vertices.size(); i++) {
        res[i] = co[el.vertices[i]];
    }
    return res;
}


LinV Geometry::asLinV(const Element& el, const std::vector<Coordinate>& co) {
    if (el.vertices.size() != 2) {
        throw std::logic_error("Invalid conversion from element to LinV");
    }
    LinV res;
    for (std::size_t i = 0; i < el.vertices.size(); i++) {
        res[i] = co[el.vertices[i]];
    }
    return res;
}

bool Geometry::approximatelyAligned(
    const TriV& a, const TriV& b, const double& approxAngle) {
    const double pi = atan(1) * 4.0;

    VecD nA = (a[1] - a[0]) ^ (a[2] - a[0]);
    VecD nB = (b[1] - b[0]) ^ (b[2] - b[0]);
    double angle = nA.angle(nB);
    if (angle < approxAngle || angle >(pi - approxAngle)) {
        return true;
    }
    return false;
}

bool Geometry::approximatelyOrientedAligned(
    const TriV& a, const TriV& b, const double& approxAngle) {
    const double pi = atan(1) * 4.0;

    VecD nA = (a[1] - a[0]) ^ (a[2] - a[0]);
    VecD nB = (b[1] - b[0]) ^ (b[2] - b[0]);
    double angle = nA.angle(nB);
    if (angle < approxAngle) {
        return true;
    }
    return false;
}

bool Geometry::areCollinear(const Coordinates& inPts) 
{
    Coordinates pts = inPts;
    std::size_t turns = 0;
    while (isDegenerate(TriV{ pts[0], pts[1], pts[2] })) {
        std::rotate(pts.begin(), pts.begin() + 1, pts.end());
        if (turns == pts.size() - 1) {
            return true;
        }
        turns++;
    }
    return false;
}

VecD Geometry::getNormal(const Coordinates& inPts, double coplanarityAngleTolerance)
{
    Coordinates pts = inPts;
    if (pts.size() < 3) {
        throw std::runtime_error("Unable to find normal for less than three points");
    }

    std::size_t turns = 0;
    while (isDegenerate(TriV{ pts[0], pts[1], pts[2] })) {
        std::rotate(pts.begin(), pts.begin() + 1, pts.end());
        if (turns == pts.size() - 1) {
            auto area = Geometry::normal(TriV{ pts[0], pts[1], pts[2] }).norm();
            throw std::runtime_error("All points are collinear.");
        }
        turns++;
    }

    const TriV seed{ pts[0], pts[1], pts[2] };
    VecD res = normal(seed);
    for (std::size_t i = 3; i < pts.size(); i++) {
        const TriV newTri{ pts[0], pts[1], pts[i] };
        if (isDegenerate(TriV{ newTri })) {
            continue;
        }
        if (!approximatelyAligned(seed, newTri, coplanarityAngleTolerance)) {
            throw std::runtime_error("Points are not coplanar.");
        }
        if (normal(newTri).norm() > res.norm()) {
            res = normal(newTri);
        }
    }

    return res / res.norm();
}

VecD Geometry::getLSFPlaneNormal(const Coordinates& inPts)
{
    //If you have n data points(x[i], y[i], z[i]), compute the 3x3 symmetric matrix A whose entries are :

    //    sum_i x[i] * x[i], sum_i x[i] * y[i], sum_i x[i]
    //    sum_i x[i] * y[i], sum_i y[i] * y[i], sum_i y[i]
    //           sum_i x[i],        sum_i y[i],     n 
    //    Also compute the 3 element vector b :
    //
    // 
    //{sum_i x[i] * z[i], sum_i y[i] * z[i], sum_i z[i]}


    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    
    double sumZ = 0.0;
    double sumXZ = 0.0;
    double sumYZ = 0.0;

    for (auto& point : inPts) {
        sumX += point[X];
        sumY += point[Y];
        sumXX += point[X] * point[X];
        sumXY += point[X] * point[Y];
        sumYY += point[Y] * point[Y];

        sumZ += point[Z];
        sumXZ += point[X] * point[Z];
        sumYZ += point[Y] * point[Z];
    }

    std::array<std::array<double, 3>, 3> matrix_A({ 
        std::array<double, 3>({sumXX, sumXY, sumX}),
        std::array<double, 3>({sumXY, sumYY, sumY}),
        std::array<double, 3>({sumX, sumY, (double) inPts.size()})
    });

    std::array<double, 3> vector_b({ sumXZ, sumYZ, sumZ });

    std::array<std::array<double, 3>, 3> transpose_A = matrix_A;

    std::array<std::array<double, 3>, 3> atA;

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double sum = matrix_A[i][0] * transpose_A[0][j] + matrix_A[i][1] * transpose_A[1][j] + matrix_A[i][2] * transpose_A[2][j];
            atA[i][j] = sum;
        }
    }
    std::array<double, 3> atb;

    for (std::size_t i = 0; i < 3; ++i) {
        double sum = transpose_A[i][0] * vector_b[0] + transpose_A[i][1] * vector_b[1] + transpose_A[i][2] * vector_b[2];
        atb[i] = sum;
    }


    // [AtA00, AtA01, AtA02]         [Atb0]
    // [AtA10, AtA11, AtA12] * x =   [Atb1]
    // [AtA20, AtA21, AtA22]         [Atb2]

    // [AtA00, AtA01, AtA02]   [x]   [Atb0]
    // [AtA10, AtA11, AtA12] * [y] = [Atb1]
    // [AtA20, AtA21, AtA22]   [z]   [Atb2]
    // 
    // AtA00 * x + AtA01 * y + AtA02 * z = Atb0
    // AtA10 * x + AtA11 * y + AtA12 * z = Atb1
    // AtA20 * x + AtA21 * y + AtA22 * z = Atb2
    // 
    // 
    // (
    // 
    // 
    // 
    // 
    //




    throw std::runtime_error("Not implemented");
    VecD res({ 0.0, 0.0, 0.0 });
    return res / res.norm();
}

VecD Geometry::getMeanNormalOfElements(
    const ElementsView& elements,
    const Coordinates& coords)
{
    VecD normal;
    for (const auto& el : elements) {
        normal += Geometry::normal(Geometry::asTriV(*el, coords));
    }
    return normal / (double) elements.size();
}

VecD Geometry::normal(const TriV& a) 
{
    return (a[1] - a[0]) ^ (a[2] - a[0]);
}

VecD Geometry::getCentroid(
    const Element& elem, const std::vector<Coordinate>& coords) 
{
    VecD res;
    for (auto const& vId : elem.vertices) {
        res += coords[vId] / (double) elem.vertices.size();
    }
    return res;
}

VecD Geometry::getCentroid(
    const TriV& tri) 
{
    VecD res;
    for (auto const& v : tri) {
        res += v / (double) tri.size();
    }
    return res;
}

bool Geometry::isDegenerate(const TriV& tri, const double& areaTolerance)
{
    return area(tri) < areaTolerance;
}

double Geometry::area(const TriV& tri) {
    return ((tri[0] - tri[1]) ^ (tri[1] - tri[2])).norm() / 2.0;
}


}
}
