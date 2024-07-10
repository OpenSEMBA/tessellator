#pragma once

#include "types/Mesh.h"
#include "Types.h"
#include "Tools.h"

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/function.hpp>

#include <iostream>

namespace meshlib {
namespace utils {

using namespace boost;

struct VertexId { 
    CoordinateId id; 
    bool operator==(const VertexId& rhs) const { return id == rhs.id; }
    bool operator<(const VertexId& rhs) const { return id < rhs.id; }
};

typedef adjacency_list<vecS, vecS, bidirectionalS, VertexId, property<edge_index_t, int>> graph_t;

typedef filtered_graph<
    graph_t, 
    function<bool(graph_t::edge_descriptor)>, 
    function<bool(graph_t::vertex_descriptor)> > ComponentGraph;

typedef property_map<graph_t, vertex_index_t>::type IndexMap;
typedef VertexId vertex_t;

typedef graph_traits<graph_t>::vertex_iterator vertex_iter;
class CoordGraph {
public:
    typedef std::pair<CoordinateId, CoordinateId> EdgeIds;
    typedef std::map<CoordinateId, graph_t::vertex_descriptor> VertexMap;
    typedef std::vector<CoordinateId> Path;
    typedef std::vector<Path> Paths;

    CoordGraph() = default;
    CoordGraph(const Elements& elems);
    CoordGraph(const ElementsView& elems);
    CoordGraph(const Paths& paths);

    void addVertex(const CoordinateId& id);
    void addEdge(const CoordinateId& id1, const CoordinateId& id2);
    void removeVertex(const CoordinateId& id);
    void removeEdge(const CoordinateId& id1, const CoordinateId& id2);
    std::size_t verticesSize() const { return num_vertices(graph_); }
    std::size_t edgesSize() const { return num_edges(graph_); }

    std::vector<CoordGraph> split() const;

    IdSet getVertices() const;
    std::vector<CoordinateId> getOrderedVertices() const;
    IdSet getAdjacentVertices(const CoordinateId id) const;
    IdSet getInterior() const;
    IdSet getExterior() const;
    std::pair<IdSet, IdSet> getBoundAndInteriorVertices() const;

    IdSet getClosestVerticesInSet(const CoordinateId& id, const IdSet& coordSet) const;

    Paths findCycles() const;
    bool isOrientableAndCyclic(const Path&) const;

    Paths findAcyclicPaths() const;
    std::set<EdgeIds> getAcyclicEdges() const;
    Path orderByOrientation(const Path&) const;
    Path findShortestPath(const CoordinateId& ini, const CoordinateId& end) const;

    CoordGraph getBoundaryGraph() const;
    CoordGraph getInternalGraph() const;
    CoordGraph intersect(const CoordGraph& rhs) const;
    template<typename Container> CoordGraph intersect(const Container& rhs) const;
    CoordGraph difference(const CoordGraph& rhs) const;
    Elements getEdgesAsLines() const;

    static std::vector<CoordGraph> buildFromElementsViews(
        const std::vector<ElementsView>& esV);
    bool canBeSplit() const;

private:
    VertexMap vertexMap_;
    graph_t graph_;

    std::vector<std::vector<VertexId>> findCycles_() const;
    IdSet getExtremes() const;

    static Paths removeRepeated(const Paths&);
    bool isForwardOriented(const Path&) const;
};

template<typename Container>
CoordGraph CoordGraph::intersect(const Container& rhs) const
{
    CoordGraph res;
    IdSet intersection = intersectWithIdSet(getVertices(), rhs);
    
    for (auto const& id : intersection) {
        res.addVertex(id);
        auto v = vertexMap_.find(id)->second;
        for (auto const& ei : make_iterator_range(out_edges(v, graph_))) {
            auto id2 = graph_[target(ei, graph_)].id;
            if (intersection.count(id2)) {
                res.addEdge(id, id2);
            }
        }

    }

    return res;
}

}
}
