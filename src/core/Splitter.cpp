#include "Splitter.h"

#include "utils/GridTools.h"

namespace meshlib::core {

std::size_t Splitter::splitSurfaces(Mesh& mesh) {
    std::size_t totalNewQuads = 0;

    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        std::vector<Element> newElements;
        
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            const Element& elem = mesh.groups[g].elements[e];
            if (elem.type == Element::Type::Surface) {
                // Split this surface into unit quads
                std::map<Coordinate, CoordinateId> coordMap;
                
                // Build initial coord map from existing coordinates
                for (CoordinateId i = 0; i < static_cast<CoordinateId>(mesh.coordinates.size()); ++i) {
                    coordMap[mesh.coordinates[i]] = i;
                }
                
                std::vector<Element> splitQuads = splitSurface_(
                    elem, mesh.coordinates, mesh.grid, coordMap);
                
                // Add new coordinates from coordMap
                for (const auto& [coord, id] : coordMap) {
                    if (id >= mesh.coordinates.size()) {
                        mesh.coordinates.push_back(coord);
                    }
                }
                
                newElements.insert(newElements.end(), splitQuads.begin(), splitQuads.end());
                totalNewQuads += splitQuads.size();
            } else {
                newElements.push_back(elem);
            }
        }
        
        mesh.groups[g].elements = std::move(newElements);
    }

    return totalNewQuads;
}

std::size_t Splitter::splitLines(Mesh& mesh) {
    std::size_t totalNewLines = 0;

    for (GroupId g = 0; g < mesh.groups.size(); g++) {
        std::vector<Element> newElements;
        
        for (ElementId e = 0; e < mesh.groups[g].elements.size(); e++) {
            const Element& elem = mesh.groups[g].elements[e];
            if (elem.type == Element::Type::Line) {
                // Split this line into unit grid lines
                std::map<Coordinate, CoordinateId> coordMap;
                
                // Build initial coord map from existing coordinates
                for (CoordinateId i = 0; i < static_cast<CoordinateId>(mesh.coordinates.size()); ++i) {
                    coordMap[mesh.coordinates[i]] = i;
                }
                
                std::vector<Element> splitLines = splitLine_(
                    elem, mesh.coordinates, mesh.grid, coordMap);
                
                // Add new coordinates from coordMap
                for (const auto& [coord, id] : coordMap) {
                    if (id >= mesh.coordinates.size()) {
                        mesh.coordinates.push_back(coord);
                    }
                }
                
                newElements.insert(newElements.end(), splitLines.begin(), splitLines.end());
                totalNewLines += splitLines.size();
            } else {
                newElements.push_back(elem);
            }
        }
        
        mesh.groups[g].elements = std::move(newElements);
    }

    return totalNewLines;
}

std::vector<Element> Splitter::splitLine_(
        const Element& line,
        const std::vector<Coordinate>& coords,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap) {
    std::vector<Element> unitLines;
    
    // Get the axis direction of the line
    Axis lineAxis = getLineAxis_(line, coords);
    
    // Get the grid cell bounds
    auto [minCell, maxCell] = getLineBounds_(line, coords);
    
    // Determine the fixed coordinate axes (the two axes perpendicular to lineAxis)
    Axis axis1 = (lineAxis + 1) % 3;
    Axis axis2 = (lineAxis + 2) % 3;
    
    // Get the fixed coordinate values from minCell
    CellDir fixedCoord1 = minCell(axis1);
    CellDir fixedCoord2 = minCell(axis2);
    
    // Generate unit lines for each cell along the line axis
    for (CellDir i = minCell(lineAxis); i < maxCell(lineAxis); i++) {
        Element unitLine = createUnitLine_(
            fixedCoord1, fixedCoord2, lineAxis, i, grid, coordMap);
        unitLines.push_back(unitLine);
    }
    
    return unitLines;
}

std::pair<Cell, Cell> Splitter::getLineBounds_(
        const Element& line,
        const std::vector<Coordinate>& coords) {
    Cell minCell = utils::GridTools::toCell(coords[line.vertices[0]]);
    Cell maxCell = utils::GridTools::toCell(coords[line.vertices[1]]);
    
    // Ensure minCell <= maxCell for all axes
    for (Axis d = 0; d < 3; d++) {
        if (minCell(d) > maxCell(d)) {
            std::swap(minCell(d), maxCell(d));
        }
    }
    
    return {minCell, maxCell};
}

Axis Splitter::getLineAxis_(
        const Element& line,
        const std::vector<Coordinate>& coords) {
    // Find which axis has different coordinate values (the line direction)
    Cell cell0 = utils::GridTools::toCell(coords[line.vertices[0]]);
    Cell cell1 = utils::GridTools::toCell(coords[line.vertices[1]]);
    
    for (Axis d = 0; d < 3; d++) {
        if (cell0(d) != cell1(d)) {
            return d;
        }
    }
    
    // Fallback (should not happen for valid lines)
    return 0;
}

Element Splitter::createUnitLine_(
        CellDir fixedCoord1,
        CellDir fixedCoord2,
        Axis lineAxis,
        CellDir cell,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap) {
    Axis axis1 = (lineAxis + 1) % 3;
    Axis axis2 = (lineAxis + 2) % 3;
    
    // Create 2 coordinates for the unit line
    std::array<Coordinate, 2> endpoints;
    endpoints[0] = Coordinate({0, 0, 0});
    endpoints[1] = Coordinate({0, 0, 0});
    
    // Set coordinates for each endpoint
    endpoints[0](lineAxis) = grid[lineAxis][cell];
    endpoints[0](axis1) = grid[axis1][fixedCoord1];
    endpoints[0](axis2) = grid[axis2][fixedCoord2];
    
    endpoints[1](lineAxis) = grid[lineAxis][cell + 1];
    endpoints[1](axis1) = grid[axis1][fixedCoord1];
    endpoints[1](axis2) = grid[axis2][fixedCoord2];
    
    // Get or create coordinate IDs
    std::array<CoordinateId, 2> vids;
    for (int i = 0; i < 2; i++) {
        auto it = coordMap.find(endpoints[i]);
        if (it != coordMap.end()) {
            vids[i] = it->second;
        } else {
            coordMap[endpoints[i]] = static_cast<CoordinateId>(coordMap.size());
            vids[i] = coordMap[endpoints[i]];
        }
    }
    
    Element unitLine;
    unitLine.type = Element::Type::Line;
    unitLine.vertices = {vids[0], vids[1]};
    
    return unitLine;
}

std::vector<Element> Splitter::splitSurface_(
        const Element& surface,
        const std::vector<Coordinate>& coords,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap) {
    std::vector<Element> quads;
    
    // Get the plane orientation of the surface
    auto [normalAxis, plane] = getSurfacePlane_(surface, coords);
    
    // Get the grid cell bounds
    auto [minCell, maxCell] = getSurfaceBounds_(surface, coords);
    
    // Determine the 2D axes in the plane
    Axis axis1 = (normalAxis + 1) % 3;
    Axis axis2 = (normalAxis + 2) % 3;
    
    // Generate unit quads for each cell in the bounds
    for (CellDir i = minCell(axis1); i < maxCell(axis1); i++) {
        for (CellDir j = minCell(axis2); j < maxCell(axis2); j++) {
            Element quad = createUnitQuad_(
                plane, normalAxis, i, j, grid, coordMap);
            quads.push_back(quad);
        }
    }
    
    return quads;
}

std::pair<Cell, Cell> Splitter::getSurfaceBounds_(
        const Element& surface,
        const std::vector<Coordinate>& coords) {
    Cell minCell = {0, 0, 0};
    Cell maxCell = {0, 0, 0};
    
    bool first = true;
    for (CoordinateId vid : surface.vertices) {
        Cell cell = utils::GridTools::toCell(coords[vid]);
        if (first) {
            minCell = cell;
            maxCell = cell;
            first = false;
        } else {
            for (Axis d = 0; d < 3; d++) {
                minCell(d) = std::min(minCell(d), cell(d));
                maxCell(d) = std::max(maxCell(d), cell(d));
            }
        }
    }
    
    // maxCell represents the cell containing the max coordinate value
    // For a surface spanning cells 0 to N-1, the max coordinate is at grid[N]
    // So maxCell should be set to N (the cell index of the max coord), which is already correct
    // The loop below will iterate i < maxCell, giving cells 0 to maxCell-1
    // No increment needed
    
    return {minCell, maxCell};
}

std::pair<Axis, CellDir> Splitter::getSurfacePlane_(
        const Element& surface,
        const std::vector<Coordinate>& coords) {
    // Find which axis has constant coordinate (the normal axis)
    for (Axis d = 0; d < 3; d++) {
        Cell cell0 = utils::GridTools::toCell(coords[surface.vertices[0]]);
        bool allSame = true;
        for (CoordinateId vid : surface.vertices) {
            Cell cell = utils::GridTools::toCell(coords[vid]);
            if (cell(d) != cell0(d)) {
                allSame = false;
                break;
            }
        }
        if (allSame) {
            return {d, cell0(d)};
        }
    }
    
    // Fallback (should not happen for valid surfaces)
    return {0, 0};
}

Element Splitter::createUnitQuad_(
        CellDir plane,
        Axis normalAxis,
        CellDir xCell,
        CellDir yCell,
        const Grid& grid,
        std::map<Coordinate, CoordinateId>& coordMap) {
    Axis axis1 = (normalAxis + 1) % 3;
    Axis axis2 = (normalAxis + 2) % 3;
    
    // Create 4 corner coordinates for the unit quad
    std::array<Coordinate, 4> corners;
    corners[0] = Coordinate({0, 0, 0});
    corners[1] = Coordinate({0, 0, 0});
    corners[2] = Coordinate({0, 0, 0});
    corners[3] = Coordinate({0, 0, 0});
    
    // Set coordinates for each corner
    corners[0](normalAxis) = grid[normalAxis][plane];
    corners[0](axis1) = grid[axis1][xCell];
    corners[0](axis2) = grid[axis2][yCell];
    
    corners[1](normalAxis) = grid[normalAxis][plane];
    corners[1](axis1) = grid[axis1][xCell + 1];
    corners[1](axis2) = grid[axis2][yCell];
    
    corners[2](normalAxis) = grid[normalAxis][plane];
    corners[2](axis1) = grid[axis1][xCell + 1];
    corners[2](axis2) = grid[axis2][yCell + 1];
    
    corners[3](normalAxis) = grid[normalAxis][plane];
    corners[3](axis1) = grid[axis1][xCell];
    corners[3](axis2) = grid[axis2][yCell + 1];
    
    // Get or create coordinate IDs
    std::array<CoordinateId, 4> vids;
    for (int i = 0; i < 4; i++) {
        auto it = coordMap.find(corners[i]);
        if (it != coordMap.end()) {
            vids[i] = it->second;
        } else {
            coordMap[corners[i]] = static_cast<CoordinateId>(coordMap.size());
            vids[i] = coordMap[corners[i]];
        }
    }
    
    Element quad;
    quad.type = Element::Type::Surface;
    quad.vertices = {vids[0], vids[1], vids[2], vids[3]};
    
    return quad;
}

}
