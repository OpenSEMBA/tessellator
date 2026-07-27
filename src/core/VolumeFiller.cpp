#include "VolumeFiller.h"
#include "cgal/filler/Filler.h"
#include "cgal/filler/FillerTools.h"
#ifdef TESSELLATOR_EXECUTION_POLICIES
#include <execution>
#endif


namespace meshlib {
namespace core {
using namespace utils;


Elements buildTriangleElements(Coordinates& cs, const TriVs tris) 
{
	Elements r;
	for (const auto& p : tris) {
		Element e;
		e.type = Element::Type::Surface;
		e.vertices.reserve(3);
		for (const auto& v : p) {
			e.vertices.push_back(cs.size());
			cs.push_back(v);
		}
		r.push_back(e);
	}
	return r;
}
Elements buildLineElements(Coordinates& cs, const LinVs lins)
{
	Elements r;
	for (const auto& p : lins) {
		Element e;
		e.type = Element::Type::Line;
		e.vertices.reserve(2);
		for (const auto& v : p) {
			e.vertices.push_back(cs.size());		
			cs.push_back(v);
		}
		r.push_back(e);
	}
	return r;
}

VolumeFiller::VolumeFiller(const Mesh& input)
{
    Mesh m;
	mesh_.grid = input.grid;
	mesh_.groups.resize(input.groups.size());

    meshlib::cgal::filler::Filler f{input};
    mesh_ = f.getMeshFilling();
    // auto slices = f.getSlices();
    // for (auto gId{0}; gId < mesh_.groups.size(); ++gId) {

    //     for (const auto& x : { X, Y, Z }) {
    //         for (const auto& [i, slice]: slices[x]) {
    //             // const Priority pr{ getGroupPriority(gId) };
    //             const Priority pr{ 0 };
    //             auto trivs = slice.buildAllTriVs(pr, x, (meshlib::cgal::filler::Height)i);
    //             auto triEls = buildTriangleElements(
    //                 m.coordinates,
    //                 slice.buildAllTriVs(pr, x, (meshlib::cgal::filler::Height)i)
    //             );

    //             auto linvs = slice.buildAllLinVs(pr, x, (meshlib::cgal::filler::Height)i); 
    //             auto lineEls = buildLineElements(
    //                 mesh_.coordinates,
    //                 slice.buildAllLinVs(pr, x, (meshlib::cgal::filler::Height)i)
    //             );

    //         }
    //     }
    // }
    //take structured mesh
    //use filler to generate the slices: intersections of each grid plane with the mesh
    //convert the slice intersection into a ElemV with dimension n, where n is the number of points of the slice intersection
    //build a coordIdMap similar to the one created in Slicer::sliceTriangle, where instead of the intersection of a triangle with the grid planes, we have the intersection of the n-dimensional ElemV
    //Create a triangulation of the intersection of the slice with the grid planes, following also slicer::sliceTriangle
}

Mesh VolumeFiller::getMesh()
{
    return mesh_;
}



}
}