#include "ElemGraph.h"

#include <algorithm>
#include <stdexcept>
#include <assert.h>
#include <set>

namespace meshlib {
namespace utils {

std::set<ElementId> ElemGraph::getVertices() const {
    std::set<ElementId> res;
    auto verts = vertices(graph_);
    for (auto& vp = verts; vp.first != vp.second; ++vp.first) {
        res.insert(graph_[*vp.first].id);
    }
    return res;
}

std::vector<ElemGraph> ElemGraph::split() const {
    property_map<graph_e, vertex_index_t>::type index = get(vertex_index, graph_);
    std::vector<std::size_t> component(num_vertices(graph_));

    std::vector<ElemGraph> res;
    if (component.size() == 0) {
        return res;
    }

    std::size_t num = connected_components(graph_, &component[0]);
    res.resize(num);
    if (num == 1) {
        res[0] = *this;
        return res;
    }
    for (std::size_t c = 0; c < num; c++) {
        std::pair<vertex_iter, vertex_iter> vp;
        for (vp = vertices(graph_); vp.first != vp.second; ++vp.first) {
            if (component[*vp.first] == c) {
                res[c].addVertex(graph_[*vp.first].id);
            }
        }
        edge_iter ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
            double weight = get(edge_weight, graph_, *ei);
            CoordinateId id1 = graph_[index[source(*ei, graph_)]].id;
            CoordinateId id2 = graph_[index[target(*ei, graph_)]].id;
            std::size_t v1 = vertexMap_.find(id1)->second;
            std::size_t v2 = vertexMap_.find(id2)->second;
            if (component[v1] == c && component[v2] == c) {
                res[c].addEdge(id1, id2, weight);
            }
        }
    }
    return res;
}

std::vector<std::pair<CoordinateId, CoordinateId>> ElemGraph::findElementsWithWeight(
    const double splitWeight) 
{
    property_map<graph_e, vertex_index_t>::type index = get(vertex_index, graph_);
    std::vector<std::pair<CoordinateId, CoordinateId>> res;
    edge_iter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {

        double weight = get(edge_weight, graph_, *ei);
        if (weight >= splitWeight) {
            ElementId id1 = graph_[index[source(*ei, graph_)]].id;
            ElementId id2 = graph_[index[target(*ei, graph_)]].id;
            res.push_back(std::make_pair(id1, id2));
        }
    }
    return res;

}

std::vector<ElemGraph> ElemGraph::splitByWeight(
    const double splitWeight) 
{
    property_map<graph_e, vertex_index_t>::type index = get(vertex_index, graph_);
    auto auxgraph_ = graph_;
    
    edge_iter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {

        double weight = get(edge_weight, graph_, *ei);
        if (weight > splitWeight) {
            ElementId id1 = graph_[index[source(*ei, graph_)]].id;
            ElementId id2 = graph_[index[target(*ei, graph_)]].id;
            remove_edge(vertexMap_[id1], vertexMap_[id2], auxgraph_);
        }
    }
    graph_ = auxgraph_;
    return this->split();
}

ElemGraph::ElemGraph(
    const Elements& elems, 
    const Coordinates& coords)
{
    std::vector<const Element*> elemPtrs;
    elemPtrs.reserve(elems.size());
    for (auto const& e : elems) {
        elemPtrs.push_back(&e);
    }
    *this = ElemGraph(elemPtrs, coords);
}

ElemGraph::ElemGraph(
    const std::vector<const Element*>& es,
    const Coordinates& cs)
{
    if (es.size() == 0) {
        return;
    }

    for (auto it = es.begin(); it != es.end(); it++) {
        const ElementId eId = &(*it) - &es.front();
        this->addVertex(eId);
    }

    if (std::all_of(es.begin(), es.end(), [](const Element* e) {return e->isLine(); })) {
        constructEdgesFromLines(es, cs);
    }
    else if (std::all_of(es.begin(), es.end(), [](const Element* e) {return e->isTriangle(); })) {
        constructEdgesFromTriangles(es, cs);
    }
    else {
        throw std::runtime_error("All elements must be of the same type.");
    }
}

void ElemGraph::addVertex(const ElementId& id) 
{
    if (vertexMap_.find(id) == vertexMap_.end()) {
        vertexMap_.emplace(id, add_vertex(Vertex({ id }), graph_));
    }
}

void ElemGraph::addEdge(const ElementId& id1, const ElementId& id2, const double weight) 
{
    if (id1 == id2) {
        throw std::runtime_error("Edges starting and finishing in same vertex are not allowed.");
    }
    
    if (vertexMap_.count(id1) == 0) {
        addVertex(id1);
    }
    if (vertexMap_.count(id2) == 0) {
        addVertex(id2);
    }
    
    add_edge(vertexMap_[id1], vertexMap_[id2], weight, graph_);
}

void ElemGraph::removeEdge(const ElementId& id1, const ElementId& id2)
{
    if (id1 == id2) {
        throw std::runtime_error("Edges starting and finishing in same vertex are not allowed.");
    }

    remove_edge(vertexMap_[id1], vertexMap_[id2], graph_);
}

std::set<ElementId> 
ElemGraph::getAdjacentVertices(const ElementId id) const 
{
    std::set<CoordinateId> res;
    auto it = vertexMap_.find(id);
    if (it == vertexMap_.end()) {
        return res;
    }
    graph_e::vertex_descriptor v = it->second;
    for (auto const& ei: make_iterator_range(out_edges(v, graph_))) {
        res.insert(graph_[target(ei, graph_)].id);
    }
    for (auto const& ei : make_iterator_range(in_edges(v, graph_))) {
        res.insert(graph_[source(ei, graph_)].id);
    }
    return res;
}

Elements ElemGraph::getAsElements(const Elements& es) const
{
    Elements res;

    for (auto const& eId : getVertices()) {
        if (eId < es.size()) {
            res.push_back(es[eId]);
        }
    }

    return res;
}

void ElemGraph::constructEdgesFromLines(const ElementsView& es, const Coordinates& cs)
{
    assert(std::all_of(es.begin(), es.end(), [](const Element* e) { return e->isLine(); }));

    for (auto it = es.begin(); it != es.end() - 1; it++) {
        const ElementId eId1 = &(*it) - &es.front();
        for (auto itComp = it + 1; itComp != es.end(); itComp++) {
            if (!Geometry::areAdjacentLines(**it, **itComp)) {
                continue;
            }
            const ElementId eId2 = &(*itComp) - &es.front();
            
            VecD t1 = cs[es[eId1]->vertices[1]] - cs[es[eId1]->vertices[0]];
            VecD t2 = cs[es[eId2]->vertices[1]] - cs[es[eId2]->vertices[0]];

            const double pi = atan(1) * 4.0;
            this->addEdge(eId1, eId2, t1.angle(t2) * 360.0 / (2.0 * pi));
        }
    }
}

using Adjacency = std::pair<ElementId, ElementId>;

std::vector<Adjacency> buildTrianglesAdjacenciesList(const ElementsView& es)
{
    assert(std::all_of(es.begin(), es.end(), [](const Element* e) { return e->isTriangle(); }));
    
    std::multimap<std::pair<CoordinateId, CoordinateId>, ElementId> sidesToElems;
    for (auto const& e : es) {
        for (std::size_t i = 0; i < e->vertices.size(); i++) {
            std::pair<CoordinateId, CoordinateId> ids({ e->vertices[i], e->vertices[(i + 1) % 3] });
            if (ids.first > ids.second) {
                std::swap(ids.first, ids.second);
            }
            sidesToElems.emplace(ids, &e - &es.front());
        }
    }

    std::vector<Adjacency> res;
    res.reserve(sidesToElems.size() / 2);
    for (auto it = sidesToElems.begin(); it != sidesToElems.end(); ++it) {
        auto nIt = std::next(it);
        if (nIt == sidesToElems.end() || it->first != nIt->first) {
            continue;
        }
        ElementId eId1 = it->second;
        ElementId eId2 = nIt->second;
        if (!Geometry::areAdjacentWithSameTopologicalOrientation(
            *es[eId1], *es[eId2])) {
            continue;
        }
        res.push_back(std::make_pair(eId1, eId2));
    }
    return res;
}


void ElemGraph::constructEdgesFromTriangles(const ElementsView& elems, const Coordinates& coords)
{      
    for (auto const& adj: buildTrianglesAdjacenciesList(elems)) {
        
        
        ElementId eId1 = adj.first;
        ElementId eId2 = adj.second;
        const double pi = atan(1) * 4.0;
        double angle = Geometry::normal(Geometry::asTriV(*elems[eId1], coords))
            .angle(Geometry::normal(Geometry::asTriV(*elems[eId2], coords)));
        this->addEdge(eId1, eId2, angle * 360.0 / (2.0 * pi));
    }
}

}
}