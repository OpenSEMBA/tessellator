#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace core {

class Staircaser : public utils::GridTools {
public:
    Staircaser(const Mesh&);
    Mesh getMesh();
    Mesh getSelectiveMesh(const std::set<Cell>& cellSet);

    Cell calculateStaircasedCell(const Relative& relative) const;
    void setFillerType(const std::string& type);


private:
    Mesh mesh_;

    Mesh inputMesh_;

    std::string fillerType_;

    using RelativePairSet = std::set<std::pair<Relative, Relative>>;

    void processTriangleAndAddToGroup(const Element& triangle, const Relatives& originalRelatives, Group& group);
    void processLineAndAddToGroup(
        const Element& line,
        const Relatives& originalRelatives,
        Relatives& resultRelatives,
        Group& group
    );
    bool isEdgePartOfCellSurface(const Element& edge, const RelativeIds &surfaceRelativeIds) const;
    bool isPureDiagonal(const Element& edge, const Relatives& relatives);
    bool isRelativeInCellsVector(const Relative& relative, const std::vector<Cell>& projectedCells) const;
    bool isRelativeInCell(const Relative& relative, const Cell& cell) const;
    void filterSurfacesFromRelativeIds(
        const RelativeIds& triangleVertices,
        int pureDiagonalIndex,
        const Relatives& originalRelatives,
        const std::map<Surfel, IdSet>& idSetByCellSurface,
        Relatives& staircasedRelatives,
        std::map<Surfel, RelativeIds>& relativeIdsByCellSurface
    );
    void addNewRelativeToGroupUsingBarycentre(
        const RelativeIds& triangleVertices,
        const Relatives& originalRelatives,
        Relatives& staircasedRelatives,
        Group& group);
    std::size_t calculateDifferenceBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Axis> calculateDifferentAxesBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Axis> calculateEqualAxesBetweenCells(const Cell& firstCell, const Cell& secondCell);
    std::vector<Cell> calculateMiddleCellsBetweenTwoRelatives(Relative& startExtreme, Relative& endExtreme);
    void calculateRelativeIdSetByCellSurface(const Relatives& relatives, std::map<Surfel, IdSet>& relativesByCellSurface);
    void fillGaps(const RelativePairSet boundaryCoordinatePairs);
};


}
}