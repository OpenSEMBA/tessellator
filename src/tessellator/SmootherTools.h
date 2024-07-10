#pragma once

#include "utils/CoordGraph.h"
#include "utils/GridTools.h"
#include "utils/Tools.h"

#include "types/Mesh.h"

#include <mutex>

namespace meshlib {
namespace tessellator {
class SmootherTools : public utils::GridTools {
public:

    class SingularIds {
    public:
        SingularIds(const IdSet& featureIds, const IdSet& contourIds, const IdSet& cornerIds) :
            featureIds_(featureIds.begin(), featureIds.end()),
            contourIds_(contourIds.begin(), contourIds.end()),
            cornerIds_(cornerIds.begin(), cornerIds.end())
        {
            edgeIds_.insert(featureIds.begin(), featureIds.end());
            edgeIds_.insert(contourIds.begin(), contourIds.end());
        }
        const IdSet& featureIds() const { return featureIds_; }
        const IdSet& contourIds() const { return contourIds_; }
        const IdSet& cornerIds()  const { return cornerIds_; }
        const IdSet& edgeIds()    const { return edgeIds_; }
    private:
        IdSet featureIds_;
        IdSet contourIds_;
        IdSet cornerIds_;
        IdSet edgeIds_;
    };

    SmootherTools(const Grid& grid);

    void collapsePointsOnFeatureEdges(
        Coordinates& res,
        const ElementsView& patch,
        const SingularIds& singularIds);

    Coordinates collapsePointsOnContour(
        const Elements& elems,
        const Coordinates& coords,
        const double alignmentThresholdAngle);
    
    void collapsePointsOnCellEdges(
        Coordinates& res,
        const ElementsView& patch,
        const SingularIds& singularIds ,
        double alignmentAngle);

    void collapsePointsOnCellFaces(
        Coordinates& res,
        const ElementsView& patch,
        const SingularIds&);

    void remeshBoundary(
        Elements& es,
        Coordinates& cs,
        const Coordinates& meshCs,
        const ElementsView& patch);

    SingularIds buildSingularIds(
        const Elements& es,
        const Coordinates& cs,
        double smoothSetAngle) const;

    void collapseInteriorPointsToBound(
        Coordinates& coords,
        const ElementsView& patch);

    void remeshElementsToOneInteriorPoint(
        Elements& es,
        Coordinates& cs,
        const ElementsView& patch);

    void remeshWithNoInteriorPoints(
        Elements& es,
        const Coordinates& cs,
        const ElementsView& patch);

private:
    std::mutex writingCoordinates_;
    std::mutex writingElements_;

    void updateCoordinates(Coordinates& res, std::map<CoordinateId, Coordinate> toMove);

    static CoordinateId getClosestEndOfPaths(
        const std::vector<utils::CoordGraph::Path>& paths);

    static Coordinate closestByDistance(
        const Coordinates& coord,
        const CoordinateId& id,
        const IdSet& candidates);

    Elements buildBoundaryContourAsLines(
        const Coordinates& coords, 
        const ElementsView& patch) const;

    static utils::CoordGraph::Path pathFromIdToAnyTarget(
        const CoordinateId& startId,
        const utils::CoordGraph::Path& cycle,
        bool forward,
        const IdSet& target);

    static IdSet getClosestValidByDistanceInCycle(
        const CoordinateId& id,
        const utils::CoordGraph::Path& cycle,
        const IdSet& valid);

    static void collapseElementsInPatch(
        Elements& es,
        const ElementsView& p,
        const CoordinateId& id,
        const CoordinateId& newId);

    static bool hasWrongOrientation(
        const Element& ref,
        const Element& check,
        const Coordinates& coords);

    static void reorient(
        Elements& es);

    static bool patchIsPlanar(
        const Coordinates& cs,
        const ElementsView& patch);


};

}
}