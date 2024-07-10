#pragma once

#include "Types.h"
#include "types/CellIndex.h"

namespace meshlib {
namespace utils {

class GridTools {
public:
    enum AxisValue {
        x = 0,
        y = 1,
        z = 2
    };

    GridTools() = default;
    GridTools(const Grid& grid);
    virtual ~GridTools() = default;

    Grid& grid();
    const Grid& grid() const;

    CellDir numCellsDir(const Axis&) const;
    Cell    numCells() const;

    std::pair<CoordinateDir, CoordinateDir> getBoundsDir(const Axis&) const;
    std::pair<Coordinate,    Coordinate>    getBounds() const;
    Cell getOffsetWithGrid(const Grid&) const;

    CoordinateDir                getStepDir(const CellDir&, const Axis&) const;
    std::array<CoordinateDir, 3> getStep   (const Cell&) const;
    CoordinateDir                getStepDir(const RelativeDir&,
                                            const Axis&) const;
    std::array<CoordinateDir, 3> getStep   (const Relative&) const;

    CoordinateDir getPosDir(const CellDir&, const Axis&) const;
    Coordinate    getPos   (const Cell&) const;
    CoordinateDir getPosDir(const RelativeDir&, const Axis&) const;
    Coordinate    getPos   (const Relative&) const;

    CellDir getCellDir(const CoordinateDir&, const Axis&) const;
    Cell    getCell   (const Coordinate&) const;

    RelativeDir getRelativeDir(const CoordinateDir&, const Axis&) const;
    Relative    getRelative   (const Coordinate&) const;
    RelativeDir getRelativeDir(const CoordinateDir&, const Axis&,
                               const CellDir&) const;
    Relative    getRelative   (const Coordinate&, const Cell&) const;

    static CellDir toCellDir(const RelativeDir&);
    static Cell    toCell   (const Relative&);

    static RelativeDir toRelativeDir(const CellDir&);
    static Relative    toRelative   (const Cell&);

    static CellDir  toNearestVertexDir(const RelativeDir&);
    static Cell     toNearestVertex   (const Relative&);

    Coordinates relativeToAbsolute(const Relatives&) const;
    Relatives absoluteToRelative(const Coordinates&) const;

    Grid getExtendedDualGrid() const;


    static bool approxDir(const CoordinateDir&,
                          const CoordinateDir&,
                          const CoordinateDir& = 1e-9);
    static bool approx   (const Coordinate&,
                          const Coordinate&,
                          const CoordinateDir& = 1e-9);

    static bool isRelativeOnCellCorner      (const Relative&);
    static bool isRelativeOnCellEdge        (const Relative&);
    static bool isRelativeOnCellFace               (const Relative&);
    static bool isRelativeOnCellFaceOrContour      (const Relative&);
    static bool isRelativeInterior                 (const Relative&);

    std::set<Cell> getTouchingCells(const Relative&) const;
    static std::size_t countIntersectingPlanes(const Relative&);
    bool sameCellProperties(const Relative&, const Relative&) const;
    
    static bool areCoordOnSameFace(const Relative& r1, const Relative& r2);
    static bool areCoordOnSameEdge(const Relative& r1, const Relative& r2);
    static std::pair<bool,Axis> getCellEdgeAxis(const Relative&);
    static std::pair<bool,Axis> getCellFaceAxis(const Relative&);

    Axis getSegmentAxisOnEdge(const Relative& r1, const Relative& r2) const;
    Axis getSegmentAxisOnFace(const Relative& r1, const Relative& r2) const;
    bool isSegmentOnEdge(const Relative& r1, const Relative& r2) const;
    bool isSegmentOnFace(const Relative& r1, const Relative& r2) const;

    static std::vector<double> linspace(double ini, double end, std::size_t num);


    std::vector<std::pair<Plane, LinV>> getEdgeIntersectionsWithPlanes(const TriV&) const;

    std::map<Cell, std::vector<const Element*>> buildCellElemMap(
        const std::vector<Element>& elems,
        const std::vector<Coordinate>& coords) const;
    std::map<Cell, std::vector<Coordinate*>> buildCellCoordMap(
        std::vector<Coordinate>& coords) const;
    std::map<Cell, std::vector<const Element*>> buildCellTriMap(
        const std::vector<Element>& elems,
        const std::vector<Coordinate>& coords) const;

    bool elementCrossesGrid(const Element&, const Coordinates&) const;

    static Grid buildCartesianGrid(double ini, double end, std::size_t num);

private:
    Grid grid_;
};

}
}

