#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace core {

class Structurer : public utils::GridTools {
public:
    Structurer(const Mesh&);
    Mesh getMesh() const { return mesh_; };

    Cell calculateStructuredCell(const Coordinate& relativeCoordinate) const;

private:
    Mesh mesh_;

    void processTriangleAndAddToGroup(const Element& triangle, const Coordinates& originalRelativeCoordinates, Group& group);
    void processLineAndAddToGroup(
        const Element& line,
        const Coordinates& originalRelativeCoordinates,
        Coordinates& resultCoordinates,
        Group& group
    );
    bool isEdgePartOfCellSurface(const Element& edge, const CoordinateIds &surfaceCoordinateIds) const;
    bool isPureDiagonal(const Element& edge, const Coordinates& coordinates);
    bool isRelativeInCellsVector(const Relative& relative, const std::vector<Cell>& projectedCells) const;
    void filterSurfacesFromCoordinateIds(
        const CoordinateIds& triangleVertices,
        int pureDiagonalIndex,
        const Coordinates& originalRelativeCoordinates,
        const std::map<Surfel, IdSet>& idSetByCellSurface,
        Coordinates& structuredCoordinates,
        std::map<Surfel, CoordinateIds>& coordinateIdsByCellSurface
    );
    void addNewCoordinateToGroupUsingBarycentre(
        const CoordinateIds& triangleVertices,
        const Coordinates& originalRelativeCoordinates,
        Coordinates& structuredCoordinates,
        Group& group);
    std::size_t calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Axis> calculateDifferentAxesBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Axis> calculateEqualAxesBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Cell> calculateMiddleCellsBetweenTwoCoordinates(Coordinate& startExtreme, Coordinate& endExtreme);
    void calculateCoordinateIdSetByCellSurface(const Coordinates& coordinates, std::map<Surfel, IdSet>& coordinatesByCellSurface);
};


}
}