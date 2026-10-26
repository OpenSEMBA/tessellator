#pragma once

#include <array>
#include <vector>

#include "types/Mesh.h"
#include "utils/Types.h"

namespace meshlib {
namespace core {

// Builds the thin-slot representations requested by the staircase mesher's
// "thinSlot" option. All meshes passed to this class must be expressed in
// relative grid coordinates.
class ThinSlotBuilder {
public:
    // A directed unit grid edge. The coordinates are grid vertices.
    using Segment = std::array<Cell, 2>;
    using Components = std::vector<std::vector<Segment>>;

    ThinSlotBuilder() = delete;

    // Splits line elements into unit grid edges and groups them into connected
    // components. Each component is ordered so that consecutive segments share
    // a vertex. Throws when the line is not staircased on the grid or branches.
    static Components buildOrderedUnitSegments(const Mesh& lineMesh);

    // Builds a local, cell-aligned view of the host surface around the given
    // segments. Compressed axis-aligned quads are expanded into unit quads.
    // The host mesh itself is not modified.
    static Mesh buildCellAlignedSurface(
        const Mesh& hostSurface,
        const Components& components);

    // Throws when a segment is not embedded in the cell-aligned surface or
    // when it lies on the surface perimeter and allowPerimeter is false.
    static void validateEmbedding(
        const Mesh& cellAlignedSurface,
        const Components& components,
        bool allowPerimeter);

    // Builds a one-quad-wide strip of cell-aligned surface quads following the
    // segments. The returned mesh is a single group and is expressed in
    // relative coordinates.
    static Mesh buildSurfaceStrip(
        const Mesh& cellAlignedSurface,
        const Components& components);
};

}
}
