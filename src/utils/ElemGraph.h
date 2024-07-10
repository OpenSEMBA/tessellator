#pragma once

#include "types/Mesh.h"
#include "Geometry.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <iostream>

namespace meshlib {
namespace utils {

using namespace boost;

struct Vertex { 
    ElementId id; 
};

typedef property<edge_weight_t, double> EdgeWeight;


typedef adjacency_list<vecS, vecS, undirectedS, Vertex, EdgeWeight> graph_e;

typedef Vertex vertex_e;
typedef graph_traits<graph_e>::vertex_iterator vertex_iter;
typedef graph_traits<graph_e>::edge_iterator edge_iter;


class ElemGraph {
public:
    typedef std::map<std::pair<ElementId, ElementId>,
        std::vector<graph_e::edge_descriptor>> EdgeMap;
    typedef std::map<ElementId, graph_e::vertex_descriptor> VertexMap;

    ElemGraph() = default;
    ElemGraph(const std::vector<const Element*>& elems, const Coordinates& coords);
    ElemGraph(const Elements& elems, const Coordinates& coords);

    void addVertex(const ElementId& id);
    void addEdge(const ElementId& id1, const ElementId& id2, const double weight);
    void removeEdge(const ElementId& id1, const ElementId& id2);

    std::vector<ElemGraph> splitByWeight(const double weight);
    std::vector<ElemGraph> split() const;

    std::vector<std::pair<CoordinateId, CoordinateId>> findElementsWithWeight(
        const double splitWeight);


    std::set<ElementId> getVertices() const;
    std::set<ElementId> getAdjacentVertices(const ElementId id) const;
    
    Elements getAsElements(const Elements&) const;
private:
    VertexMap vertexMap_;
    graph_e graph_;

    void constructEdgesFromLines(const ElementsView&, const Coordinates&);
    void constructEdgesFromTriangles(const ElementsView&, const Coordinates&);

};

}
}
