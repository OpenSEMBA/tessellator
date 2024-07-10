#include "CoordGraph.h"

#include <algorithm>
#include <stdexcept>
#include <assert.h>
#include <set>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/tiernan_all_cycles.hpp>

namespace boost { template <typename G> inline void renumber_vertex_indices(G& g) { throw; } }

namespace meshlib {
namespace utils {

IdSet CoordGraph::getVertices() const 
{
    IdSet res;
    auto verts = vertices(graph_);
    for (auto& vp = verts; vp.first != vp.second; ++vp.first) {
        res.insert(graph_[*vp.first].id);
    }
    return res;
}
std::vector<CoordinateId> CoordGraph::getOrderedVertices() const 
{
    std::vector<CoordinateId> res;
    auto verts = vertices(graph_);
    for (auto& vp = verts; vp.first != vp.second; ++vp.first) {
        res.push_back(graph_[*vp.first].id);
    }
    return res;
}

bool CoordGraph::canBeSplit() const
{
    std::vector<std::size_t> component(num_vertices(graph_));
    std::size_t num = connected_components(graph_, &component[0]);
    return num > 1;
}

std::vector<CoordGraph> CoordGraph::split() const 
{
    if (num_vertices(graph_) == 0) {
        return {};
    }

    std::vector<std::size_t> component(num_vertices(graph_));
    std::size_t num = connected_components(graph_, &component[0]);
    if (num == 1) {
        return { *this };
    }

    IndexMap index = get(vertex_index, graph_);
    std::vector<CoordGraph> res(num);
    for (std::size_t c = 0; c < num; c++) {
        graph_t::edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
            CoordinateId id1 = graph_[index[source(*ei, graph_)]].id;
            CoordinateId id2 = graph_[index[target(*ei, graph_)]].id;
            std::size_t v1 = vertexMap_.find(id1)->second;
            std::size_t v2 = vertexMap_.find(id2)->second;
            if (component[v1] == c && component[v2] == c) {
                res[c].addEdge(id1, id2);
            }
        }
    }
    return res;
}

CoordGraph::CoordGraph(const Elements& elems) 
{
    std::vector<const Element*> elemPtrs;
    elemPtrs.reserve(elems.size());
    for (auto const& e : elems) {
        elemPtrs.push_back(&e);
    }
    *this = CoordGraph(elemPtrs);
}

CoordGraph::CoordGraph(const ElementsView& es)
{
    for (auto const& e : es) {
        for (std::size_t i = 0; i < e->vertices.size(); i++) {
            this->addEdge(
                e->vertices[i], 
                e->vertices[(i + 1) % e->vertices.size()]
            );
            if (e->vertices.size() <= 2) {
                break;
            }
        }
    }
}

CoordGraph::CoordGraph(const Paths& paths) {
    for (const auto& p : paths) {
        for (std::size_t i = 0; i < p.size(); i++) {
            this->addEdge(
                p[i],
                p[(i + 1) % p.size()]
            );
            if (p.size() <= 2) {
                break;
            }
        }
    }
}

void CoordGraph::addVertex(const CoordinateId& id) 
{
    if (vertexMap_.find(id) == vertexMap_.end()) {
        vertexMap_.emplace(id, add_vertex(VertexId({ id }), graph_));
    }
}

void CoordGraph::addEdge(const CoordinateId& id1, const CoordinateId& id2) 
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
    
    add_edge(vertexMap_[id1], vertexMap_[id2], graph_);
}

void CoordGraph::removeVertex(const CoordinateId& id1)
{
    remove_vertex(vertexMap_[id1], graph_);
}

void CoordGraph::removeEdge(const CoordinateId& id1, const CoordinateId& id2)
{
    if (id1 == id2) {
        throw std::runtime_error("Edges starting and finishing in same vertex are not allowed.");
    }

    remove_edge(vertexMap_[id1], vertexMap_[id2], graph_);
}

IdSet CoordGraph::getAdjacentVertices(const CoordinateId id) const 
{
    IdSet res;
    auto it = vertexMap_.find(id);
    if (it == vertexMap_.end()) {
        return res;
    }
    graph_t::vertex_descriptor v = it->second;
    for (auto const& ei: make_iterator_range(out_edges(v, graph_))) {
        res.insert(graph_[target(ei, graph_)].id);
    }
    for (auto const& ei : make_iterator_range(in_edges(v, graph_))) {
        res.insert(graph_[source(ei, graph_)].id);
    }
    return res;
}

IdSet CoordGraph::getInterior() const 
{
    IdSet interior;
    for (auto const& v : getVertices()) {
        IdSet adjVertices = getAdjacentVertices(v);
        if (adjVertices.size() == 0 || adjVertices.size() == 1) {
            continue;
        } else if (adjVertices.size() == 2) {
            interior.insert(v);
        } else {
            throw std::runtime_error("getInterior @ CoordGraph: cannot find interior");
        }
    }
    return interior;
}

IdSet CoordGraph::getExterior() const 
{
    IdSet exterior;
    for (auto const& v : getVertices()) {
        IdSet adjVertices = getAdjacentVertices(v);
        if  (adjVertices.size() == 0 || adjVertices.size() == 1) {
            exterior.insert(v);
        } else if (adjVertices.size() == 2) {
            continue;
        }
        else {
            throw std::runtime_error("getExterior @ CoordGraph: cannot find exterior");
        }
    }
    return exterior;
}

IdSet CoordGraph::getClosestVerticesInSet(
    const CoordinateId& vI, 
    const IdSet& ids) const
{
    assert(ids.count(vI) == 0);
    
    struct cmp {
        bool operator() (const Path& a, const Path& b) const {
            if (a.size() < b.size()) {
                return true;
            }
            else if (a.size() == b.size()) {
                return a < b;
            }
            else {
                return false;
            }
        }
    };

    std::set<Path, cmp> paths;
    for (auto const& vB : ids) {
        if (vertexMap_.find(vB) == vertexMap_.end()) {
            continue;
        }
        auto path = findShortestPath(vI, vB);
        if (path.empty()) {
            throw std::runtime_error("Can not find path to point in set.");
        }
        paths.insert(path);
    }
    
    if (paths.empty()) {
        return {};
    }

    const std::size_t shortestPathSize = paths.begin()->size();
    IdSet res;
    for (auto const& path : paths) {
        if (path.size() == shortestPathSize) {
            res.insert(path.back());
        }
        else {
            break;
        }
    }
    return res;
}


bool CoordGraph::isOrientableAndCyclic(const Path& path) const 
{
    if (path.empty()) {
        return false;
    }
    if (isForwardOriented(path) || isForwardOriented(Path(path.rbegin(), path.rend())) ){
        IdSet idsInPath(path.begin(), path.end());
        auto v = vertexMap_.find(*idsInPath.begin())->second;
        bool nextVertexFound = true;
        while (!idsInPath.empty()) {
            nextVertexFound = false;

            graph_traits<graph_t>::out_edge_iterator ei, ei_end;
            for (tie(ei, ei_end) = out_edges(v, graph_); ei != ei_end; ++ei) {
                auto tgtId = graph_[target(*ei, graph_)].id;
                if (idsInPath.count(tgtId)) {
                    nextVertexFound = true;
                    v = target(*ei, graph_);
                    idsInPath.erase(tgtId);
                    break;
                }
            }

            if (!nextVertexFound) {
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }

}

IdSet CoordGraph::getExtremes() const 
{
    IdSet extremes;
    for (auto vp = vertices(graph_); vp.first != vp.second; ++vp.first) {
        graph_t::vertex_iterator vIt = vp.first;
        graph_t::vertex_descriptor v = *vIt;
        CoordinateId vId = graph_[*vp.first].id;
        if (getAdjacentVertices(vId).size() == 1) {
            extremes.insert(vId);
        }
    }
    return extremes;
}

CoordGraph::Paths CoordGraph::removeRepeated(const Paths& paths) 
{
    std::map<std::set<CoordinateId>, Path> auxMaps;
    for (auto const& path : paths) {
        std::set<CoordinateId> key(path.begin(), path.end());
        if (auxMaps.find(key) == auxMaps.end()) {
            auxMaps.emplace(key, path);
        }
    }

    Paths res;
    for (auto const& auxMap : auxMaps) {
        res.push_back(auxMap.second);
    }
    return res;
}

CoordGraph::Path CoordGraph::orderByOrientation(const Path& path) const
{
    if (isForwardOriented(path)) {
        return path;
    }

    Path reversePath(path.rbegin(), path.rend());
    if (isForwardOriented(reversePath)) {
        return reversePath;
    }

    throw std::runtime_error("Unable to order path by orientation.");
}

bool CoordGraph::isForwardOriented(const Path& path) const
{
    for (auto it = path.begin(); it != path.end(); ++it) {
        if (it + 1 == path.end()) {
            return true;
        }
        
        CoordinateId nextIdInPath = *(it + 1);
        
        bool nextInPath = false;
        auto v = vertexMap_.find(*it)->second;
        for (auto const& ei : make_iterator_range(out_edges(v, graph_))) {
            CoordinateId targetId = graph_[target(ei, graph_)].id;
            if (targetId == nextIdInPath) {
                nextInPath = true;
                break;
            }
        }
        
        if (!nextInPath) {
            return false;
        }
    }
    throw std::runtime_error("Unable to determine if path is forward oriented.");
}

std::set<CoordGraph::EdgeIds> CoordGraph::getAcyclicEdges() const
{
    IdSet inCycles;
    for (auto const& cycle : findCycles()) {
        inCycles.insert(cycle.begin(), cycle.end());
    }

    std::set<EdgeIds> res;
    graph_t::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
        auto src = graph_[source(*ei, graph_)].id;
        auto tgt = graph_[target(*ei, graph_)].id;
        if (inCycles.count(src) && inCycles.count(tgt)) {
            continue;
        }
        res.insert({ src, tgt });
    }
    return res;
}

CoordGraph::Paths CoordGraph::findAcyclicPaths() const 
{
    IdSet inCycles;
    for (auto const& cycle : findCycles()) {
        inCycles.insert(cycle.begin(), cycle.end());
    }


    std::vector<Path> paths;
    for (auto const& extreme: getExtremes()) {
        paths.push_back({});
        bool isNeighExtreme = false;
        bool isNeighInCycle = false;
        graph_t::vertex_descriptor vCurrent = vertexMap_.find(extreme)->second;
        IdSet visited;
        while (!isNeighExtreme && !isNeighInCycle) {
            CoordinateId currentId = graph_[vCurrent].id;
            paths.back().push_back(currentId);
            visited.insert(currentId);
                
            graph_t::vertex_descriptor vNeigh = 0;
            for (auto const& adj: getAdjacentVertices(currentId)) {
                if (visited.count(adj) != 1) {
                    auto it = vertexMap_.find(adj);
                    vNeigh = it->second;
                    break;
                }
            }
            CoordinateId neighId = graph_[vNeigh].id;
            
            isNeighExtreme = getAdjacentVertices(neighId).size() == 1;
            isNeighInCycle = inCycles.count(neighId) > 0;
            if (isNeighExtreme || isNeighInCycle) {
                paths.back().push_back(neighId);
            }
            vCurrent = vNeigh;
        }
    }

    return removeRepeated(paths);
}

std::pair<IdSet, IdSet> CoordGraph::getBoundAndInteriorVertices() const 
{
    IdSet all = getVertices();
    IdSet bound = getBoundaryGraph().getVertices();
    IdSet interior;
    std::set_difference(
        all.begin(), all.end(),
        bound.begin(), bound.end(),
        std::inserter(interior, interior.begin())
    );
    return std::make_pair(bound, interior);
}

CoordGraph CoordGraph::getBoundaryGraph() const 
{
    CoordGraph res;
    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
        auto src = source(*ei, graph_);
        auto tgt = target(*ei, graph_);
        if (!edge(tgt, src, graph_).second) {
            res.addEdge(graph_[src].id, graph_[tgt].id);
        }
    }
    return res;
}

CoordGraph CoordGraph::getInternalGraph() const
{
    return difference(getBoundaryGraph());
}

CoordGraph CoordGraph::difference(const CoordGraph& rhs) const 
{
    CoordGraph diff = *this;
    graph_traits<graph_t>::edge_iterator ei, ei_end;
    IndexMap index = get(vertex_index, rhs.graph_);
    for (std::tie(ei, ei_end) = edges(rhs.graph_); ei != ei_end; ++ei) {
        CoordinateId id1 = rhs.graph_[index[source(*ei, rhs.graph_)]].id;
        CoordinateId id2 = rhs.graph_[index[target(*ei, rhs.graph_)]].id;
        diff.removeEdge(id1, id2);
    }

    CoordGraph res;
    for (std::tie(ei, ei_end) = edges(diff.graph_); ei != ei_end; ++ei) {
        CoordinateId id1 = diff.graph_[index[source(*ei, diff.graph_)]].id;
        CoordinateId id2 = diff.graph_[index[target(*ei, diff.graph_)]].id;
        res.addEdge(id1, id2);
    }

    return res;
}

CoordGraph CoordGraph::intersect(const CoordGraph& rhs) const 
{
    return intersect(rhs.getVertices());
}

CoordGraph::Path CoordGraph::findShortestPath(
    const CoordinateId& ini, 
    const CoordinateId& end) const
{
    typedef property<edge_weight_t, double> EdgeWeight;
    typedef adjacency_list<vecS, vecS, bidirectionalS, VertexId, EdgeWeight> graph_w;
    typedef property_map<graph_w, vertex_index_t>::type IndexMapW;
    typedef boost::iterator_property_map <
        graph_w::vertex_descriptor*,
        IndexMapW,
        graph_w::vertex_descriptor,
        graph_w::vertex_descriptor& > PredecessorMap;
    typedef std::map<CoordinateId, graph_w::vertex_descriptor> vertexMapW;

    graph_w graphd;
    vertexMapW vertex_map;
    
    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (std::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
        
        CoordinateId cSource = graph_[ei->m_source].id;
        CoordinateId cTarget = graph_[ei->m_target].id;

        if (vertex_map.count(cSource) == 0) {
            vertex_map.emplace(cSource, add_vertex(VertexId({ cSource }), graphd));
        }
        if (vertex_map.count(cTarget) == 0) {
            vertex_map.emplace(cTarget, add_vertex(VertexId({ cTarget }), graphd));
        }
        add_edge(vertex_map[cSource], vertex_map[cTarget], graphd);
        add_edge(vertex_map[cTarget], vertex_map[cSource], graphd);
    }

    IndexMapW index = get(vertex_index, graphd);
    graph_w::vertex_descriptor startVertex = vertex_map.find(ini)->second;

    std::vector<graph_w::vertex_descriptor> predecessors(num_vertices(graphd));
    std::vector<double> distances(num_vertices(graphd));

    PredecessorMap predecessorMap(&predecessors[0], index);

    dijkstra_shortest_paths(graphd, startVertex, predecessor_map(predecessorMap));

    Path reverse;
    CoordinateId currentId = end;
    graph_w::vertex_descriptor currentVertex = vertex_map.find(end)->second;
    while (currentVertex != startVertex) {
        currentId = graphd[currentVertex].id;
        reverse.push_back(currentId);
        currentVertex = predecessorMap[currentVertex];
        if (reverse.size() > num_vertices(graphd)) {
            return Path({});
        }
    }
    currentId = graphd[currentVertex].id;
    reverse.push_back(currentId);
    currentVertex = predecessorMap[currentVertex];

    return Path(reverse.rbegin(), reverse.rend());
}

std::vector<CoordGraph> CoordGraph::buildFromElementsViews(
    const std::vector<ElementsView>& esVs)
{
    std::vector<CoordGraph> res;
    res.resize(esVs.size());
    for (auto const& esV : esVs) {
        res.push_back(CoordGraph(esV));
    }
    return res;
}

Elements CoordGraph::getEdgesAsLines() const
{
    Elements res;

    graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei) {
        auto src = source(*ei, graph_);
        auto tgt = target(*ei, graph_);
        
        auto srcId = graph_[src].id;
        auto tgtId = graph_[tgt].id;
            
        Element line;
        line.type = Element::Type::Line;
        line.vertices = { srcId, tgtId };
        res.push_back(line);
    }

    return res;
}


struct cycle_recorder
{
    template <typename Path, typename graph_t>
    inline void cycle(const Path& p, const graph_t& g)
    { 
        cycles->push_back({});
        for (auto v : p) {
            cycles->back().push_back(g[v].id);
        }
    }
    std::vector<CoordGraph::Path>* cycles;
};

std::vector<CoordGraph::Path> CoordGraph::findCycles() const
{
    std::vector<Path> res;
    cycle_recorder vis{&res};
    tiernan_all_cycles(graph_, vis);
    return res;
}

}
}