#include "Delaunator.h"
#include "utils/Geometry.h"

namespace meshlib {
namespace cgal {

using namespace utils;

const double Delaunator::COPLANARITY_ANGLE_TOLERANCE = 0.1;

Delaunator::Delaunator(const Coordinates* global, const ElementsView& elements)
{
    if (global == nullptr) {
        throw std::runtime_error("Global list of coordinates must be defined");
    }

    globalCoords_ = global;
    if (!elements.empty()) {
        elements_ = elements;
    }
}


std::vector<Element> Delaunator::mesh(
    const IdSet& inIds,
    const std::vector<Polygon>& constrainingPolygons) const
{
    auto ids = filterIdsByConstraints(inIds, constrainingPolygons);
    checkIdsAreInRange(ids, constrainingPolygons);
    const IndexPointToId pointToId = buildPointsInIndex(ids, constrainingPolygons);
    CDT cdt = buildCDT(pointToId, ids, constrainingPolygons);
    return convertFromCDT(cdt, pointToId);
}

void Delaunator::checkIdsAreInRange(
    const IdSet& inIds,
    const std::vector<Polygon>& constrainingPolygons) const
{
    bool outOfRange = false;
    for (auto const& id : inIds) {
        if (id >= globalCoords_->size()) {
            outOfRange = true;
        }
    }
    for (auto const& p : constrainingPolygons) {
        for (auto const& id : p) {
            if (id >= globalCoords_->size()) {
                outOfRange = true;
            }
        }
    }
    if (outOfRange) {
        throw std::runtime_error("Coordinates ids are out of range.");
    }
}

std::vector<Element> Delaunator::convertFromCDT(
    const CDT& cdt, const IndexPointToId& pointToId) const
{
    for (auto v : cdt.finite_vertex_handles()) {
        if (pointToId.left.count(v->point()) == 0) {
            throw std::runtime_error("A CDT point was not in original set or constraints.");
        }
    }
    
    std::vector<Element> res;
    for (Face_handle f : cdt.finite_face_handles())
    {
        if (f->info().in_domain()) {
            Element tri;
            tri.type = Element::Type::Surface;
            for (int i = 0; i < 3; i++) {
                const Point point = f->vertex(i)->point();
                auto it = pointToId.left.find(point);
                tri.vertices.push_back(it->second);
            }
            res.push_back(tri);
        }
    }
    return res;
}

Delaunator::CDT Delaunator::buildCDT(
    const IndexPointToId& pointToId,
    const IdSet& inIds,
    const Polygons& constrainingPolygons)
{
    CDT cdt;
    for (auto const& polygon : constrainingPolygons) {
        Polygon_2 cgalPoly;
        for (auto const& id : polygon) {
            const Point& point = pointToId.right.find(id)->second;
            cgalPoly.push_back(point);
        }
        cdt.insert_constraint(cgalPoly.vertices_begin(), cgalPoly.vertices_end(), true);
    }
    for (auto const& id : inIds) {
        const Point& point = pointToId.right.find(id)->second;
        cdt.insert(point);
    }
    mark_domains(cdt);
    return cdt;
}

void Delaunator::mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border)
{
    if (start->info().nesting_level != -1) {
        return;
    }
    std::list<Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()) {
        Face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++) {
                CDT::Edge e(fh, i);
                Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1) {
                    if (ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

IdSet Delaunator::filterIdsByConstraints(
    const IdSet& ids, const Polygons& polys)
{
    IdSet res = ids;
    for (auto const& p : polys) {
        for (auto const& id : p) {
            res.erase(id);
        }
    }
    return res;
}


void Delaunator::mark_domains(CDT& cdt)
{
    for (CDT::Face_handle f : cdt.all_face_handles()) {
        f->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1) {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}

Delaunator::IndexPointToId Delaunator::buildPointsInIndex(
    const IdSet& inIds,
    const Polygons& constrainingPolygons) const
{
    Coordinates cs;
    std::vector<CoordinateId> originalIds;
    for (auto const& id : inIds) {
        cs.push_back((*globalCoords_)[id]);
        originalIds.push_back(id);
    }
    for (auto const& polygon : constrainingPolygons) {
        for (auto const& id : polygon) {
            cs.push_back((*globalCoords_)[id]);
            originalIds.push_back(id);
        }
    }
    
    VecD normal({ 0.,0.,0. });
    if (!elements_.empty() && 
        std::all_of(elements_.begin(), 
                    elements_.end(), 
                    [&](auto* el) {return el->isTriangle(); })) 
    {
        normal = utils::Geometry::getMeanNormalOfElements(elements_, *globalCoords_);
    }
    
    utils::Geometry::rotateToXYPlane(cs.begin(), cs.end(), normal);

    IndexPointToId res;
    for (std::size_t i = 0; i < cs.size(); i++) {
        res.insert( IndexPointToId::value_type(
            CDT::Point(cs[i](0), cs[i](1)), 
            originalIds[i]) 
        );
    }

    return res;
}

}
}