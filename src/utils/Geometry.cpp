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

VecD Geometry::normal(const TriV& a) {
    return (a[1] - a[0]) ^ (a[2] - a[0]);
}

VecD Geometry::getCentroid(
    const Element& elem, const std::vector<Coordinate>& coords) {
    VecD res;
    for (auto const& vId : elem.vertices) {
        res += coords[vId] / (double) elem.vertices.size();
    }
    return res;
}

VecD Geometry::getCentroid(
    const TriV& tri) {
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
