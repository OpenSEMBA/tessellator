#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

namespace meshlib::core {

class Splitter {
public:
    // Split all surfaces in mesh into unit quads (1x1 grid cells)
    // Returns number of new quads created
    static std::size_t splitSurfaces(Mesh& mesh);

private:
    // Split a single surface into unit quads
    static std::vector<Element> splitSurface_(
        const Element& surface,
        const std::vector<Coordinate>& coords,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap);
    
    // Get grid cell bounds for a surface
    static std::pair<Cell, Cell> getSurfaceBounds_(
        const Element& surface,
        const std::vector<Coordinate>& coords);
    
    // Determine the plane orientation of a surface (which axis is normal)
    static std::pair<Axis, CellDir> getSurfacePlane_(
        const Element& surface,
        const std::vector<Coordinate>& coords);
    
    // Create a unit quad at given grid cell position
    static Element createUnitQuad_(
        CellDir plane,
        Axis normalAxis,
        CellDir xCell,
        CellDir yCell,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap);
};

}
