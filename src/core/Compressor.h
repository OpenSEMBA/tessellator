#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"

#include <set>
#include <vector>

namespace meshlib::core {

class Compressor {
public:
    // Compress only quad surfaces (4 vertices) that are coplanar and adjacent
    // Returns number of surfaces merged (original_count - compressed_count)
    static std::size_t compressSurfaces(Mesh& mesh);

private:
    // Group surfaces by (grid_plane, sign, axis) and compress each group
    static std::vector<Element> compressSurfs_(
        std::vector<Coordinate>& coords,
        const std::vector<Element>& surfs);

    // Compress surfaces with same normal direction and sign
    static std::vector<Element> compressDirSignSurfs_(
        std::vector<Coordinate>& coords,
        const std::pair<Sign, Axis>& signDir,
        const std::vector<Element>& surfs);

    // Compress connected coplanar surfaces using contour detection
    static std::vector<Element> compressSurf_(
        std::vector<Coordinate>& coords,
        const std::pair<Sign, Axis>& signDir,
        const std::vector<Element>& surfs);

    // Merge adjacent surfels into maximal rectangles
    static std::vector<PlaneSurface> compressSurfels_(
        const std::set<PlaneSurfel>& surfs);

    // Detect boundary contours of surfel set
    static std::vector<Contour> getContours_(const std::set<PlaneSurfel>& surfs);

    // Trace a single contour from starting edge
    static Contour getContour_(
        const PlaneLinel& from,
        const std::set<PlaneSurfel>& surfs,
        std::set<PlaneLinel>& visited);

    // Get edge of a surfel in given direction
    static PlaneLinel getSurfaceEdge_(
        const PlaneSurfel& surf,
        const CellDir& diff,
        const Axis& dir);

    // Find lines crossing between contours
    static std::array<std::vector<CrossLine>, 2> getCrossingLines_(
        const std::set<PlaneSurfel>& surfs,
        const std::vector<Contour>& contours);

    // Select the set of crossing lines with maximum count
    static std::array<std::vector<CrossLine>, 2> getMaxCompatLines_(
        const std::array<std::vector<CrossLine>, 2>& cross);

    // Get linels between two points
    static std::set<PlaneLinel> getLinels_(
        const PlanePoint& ini,
        const PlanePoint& end);

    // Add linels at concave corners
    static void addConcaveLinels_(
        const std::set<PlaneSurfel>& surfs,
        const std::vector<Contour>& contours,
        std::set<PlaneLinel>& lines);
};

}
