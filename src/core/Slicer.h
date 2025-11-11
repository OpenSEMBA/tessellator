#pragma once

#include <set>
#include <iostream>
#include <mutex>

#include "utils/GridTools.h"

namespace meshlib {
namespace core {

struct SlicerOptions {
    int initialCollapsingDecimalPlaces = 4;
};

class Slicer : public utils::GridTools {
public:
    
    typedef std::map<Plane, IdSet> PlaneCoords;
    typedef std::map<Cell, IdSet> CellCoordIdMap;    
    typedef std::pair<Plane, Coordinates> PlaneAlignedPolyline;
    typedef std::vector<PlaneAlignedPolyline> PlaneAlignedPolylines;
    typedef std::vector<Coordinate> PolylineV;

    Slicer(const Mesh&, const std::vector<Element::Type>& dimensionPolicy = {}, const SlicerOptions& opts = SlicerOptions());
    Mesh getMesh() const { return mesh_; };


    static Elements buildTrianglesFromPath(const std::vector<Coordinate>&, const std::vector<CoordinateId>&);

private:
    std::mutex writingCoordinates_;
    std::mutex writingElements_;

    Mesh mesh_;
    SlicerOptions opts_;

    Elements sliceTriangle(Coordinates&, const TriV&);
    Elements sliceLine(Coordinates&, const LinV&);
    
    template<std::size_t N>
    IdSet buildIntersectionsWithGridPlanes(
        Coordinates& sCoords,
        const ElemV<N>& tri);
    
    CellCoordIdMap buildCellCoordIdMap(
        Coordinates& sCoords,
        const IdSet& idSet) const;

    PolylineV meshSegments(const LinV&) const;
    
    void getCellPosNext(Cell&, Coordinate&,
        const Cell&, const Coordinate&,
        const Cell&, const Coordinate&) const;

};

}
}
