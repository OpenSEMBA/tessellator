#include "Repairer.h"

#include "utils/Cleaner.h"
#include "utils/MeshTools.h"

#include "PolyhedronTools.h"

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>

namespace meshlib {
namespace cgal {

using vertex_descriptor = boost::graph_traits<Polyhedron>::vertex_descriptor;
using halfedge_descriptor = boost::graph_traits<Polyhedron>::halfedge_descriptor;
using face_descriptor = boost::graph_traits<Polyhedron>::face_descriptor;

using namespace utils;
using namespace meshTools;
using namespace polyhedronTools;

void fillHoles(Polyhedron& p)
{   
    std::vector<halfedge_descriptor> border_cycles;
    PMP::extract_boundary_cycles(p, std::back_inserter(border_cycles));
    for (halfedge_descriptor h : border_cycles)
    {
        std::vector<face_descriptor>  patch_facets;
        std::vector<vertex_descriptor> patch_vertices;
        PMP::triangulate_refine_and_fair_hole(
            p,
            h,
            std::back_inserter(patch_facets),
            std::back_inserter(patch_vertices)
        );
    }
}

void repairGroup(
    Coordinates& oCs, Group& oG, 
    const Coordinates& inCs, const Group& inG)
{
    if (inG.elements.empty()) {
        return;
    }

    Polyhedron closedP, openP;
    std::tie(closedP, openP) = buildClosedAndOpenPolyhedrons(inCs, inG.elements);
    
    auto closedElems = buildElementsFromPolyhedron(oCs, closedP);
    oG.elements.insert(oG.elements.end(), closedElems.begin(), closedElems.end());
    
    if (PMP::does_self_intersect(openP)) {
        throw std::runtime_error("Mesh contains self intersections.");
    }
    fillHoles(openP);
    assert(CGAL::is_closed(openP));
    assert(!PMP::does_self_intersect(openP));

    auto repairedElems{ buildElementsFromPolyhedron(oCs, openP) };
    oG.elements.insert(oG.elements.end(), repairedElems.begin(), repairedElems.end());
}

Mesh repair(const Mesh& m)
{
    
    auto r{ buildMeshFilteringElements(m, isTetrahedron) };
    for (const auto& g : m.groups) {
        auto gId{ &g - &m.groups.front() };
        repairGroup(r.coordinates, r.groups[gId], m.coordinates, g);
    }

    Cleaner::fuseCoords(r);
    Cleaner::cleanCoords(r);

    return r;
}



}
}