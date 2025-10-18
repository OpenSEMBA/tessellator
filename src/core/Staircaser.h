#pragma once

#include "utils/GridTools.h"

namespace meshlib {
namespace core {

class Staircaser : public utils::GridTools {
public:
    Staircaser(const Mesh&);
    Mesh getMesh();
    
    Cell calculateStaircasedCell(const Relative& relative) const;
    
    //  When structuring a specific set of cells, especially when converting a single edge into two, neighboring non
    //  structured cells may be left with topological gaps. Applying a gap-filling strategy helps maintain mesh 
    //  integrity and preserve the topology. These gaps typically involve three bounding nodes, forming a triangular shape.
    //  These options are used exclusively in the getSelectiveMesh function.
    enum class GapsFillingType {
        // None: No gap filling is applied (default). Useful for debugging.
        None,
        
        // Insert: Fills each gap by creating a triangle with its three bounding nodes.
        Insert,
        
        // Split: Finds the triangle adjacent to the gap (shares two nodes), 
        // then builds new triangles of the form {vertex1/2, neighborVertex, vertex3}, 
        // where vertex1/2 are the shared nodes and neighborVertex is a common neighbor.
        Split
    };

    Mesh getSelectiveMesh(const std::set<Cell>& cellSet, GapsFillingType type = GapsFillingType::None);

private:
    Mesh mesh_;

    Mesh inputMesh_;

    GapsFillingType fillerType_;

    using RelativePairSet = std::set<std::pair<Relative, Relative>>;

    void processTriangleAndAddToGroup(const Element& triangle, const Relatives& originalRelatives, Group& group);
    void processLineAndAddToGroup(
        const Element& line,
        const Relatives& originalRelatives,
        Relatives& resultRelatives,
        Group& group
    );
    void processNodeAndAddToGroup(
        const Element& node,
        const Relatives& originalRelative,
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