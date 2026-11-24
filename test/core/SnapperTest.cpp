#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "Snapper.h"
#include "Slicer.h"
#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

#include <algorithm>


// #if APP_LOADED
#include "app/vtkIO.h"
// #endif

using namespace meshlib;
using namespace core;
using namespace utils;
using namespace meshFixtures;

class SnapperTest : public ::testing::Test {
protected:
};

TEST_F(SnapperTest, similar_results_for_each_plane)
{
    SnapperOptions opts;
    opts.edgePoints = 7;
    opts.forbiddenLength = 0.1;

    for (auto z: {0.0, 1.0}) {

        Mesh m;
        m.grid = buildUnitLengthGrid(1.0);
        m.coordinates = {
            Relative({0.80, 1.00, z}),
            Relative({1.00, 1.00, z}),
            Relative({0.80, 0.00, z}),
            Relative({1.00, 0.00, z})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 1, 3})
        };
                
        auto res = Snapper(m, opts).getMesh();
        EXPECT_EQ(res.coordinates, m.coordinates);

    }
}

TEST_F(SnapperTest, edgeSnapsDoNotPreferExistingCoordinates)
{
    const auto snappedPoint = [](double position, const Coordinates& existing) {
        Mesh mesh;
        mesh.grid = Grid{
            std::vector<double>{ 0.0, 3.0, 5.0 },
            std::vector<double>{ 0.0, 1.0 },
            std::vector<double>{ 0.0, 1.0 }
        };
        mesh.coordinates = { Relative({ position, 0.0, 0.0 }) };
        mesh.coordinates.insert(
            mesh.coordinates.end(), existing.begin(), existing.end());
        mesh.groups = { Group() };
        for (CoordinateId i = 0; i < mesh.coordinates.size(); ++i) {
            mesh.groups[0].elements.emplace_back(
                std::vector<CoordinateId>{ i }, Element::Type::Node);
        }

        SnapperOptions options;
        options.edgePoints = 3;
        const Mesh result = Snapper(mesh, options).getMesh();
        return result.coordinates[result.groups[0].elements[0].vertices[0]];
    };

    const Coordinate first({ 1.0, 0.0, 0.0 });
    const Coordinate second({ 1.25, 0.0, 0.0 });
    EXPECT_EQ(snappedPoint(1.125, { second }), first);
    EXPECT_EQ(snappedPoint(1.125, {}), first);
    EXPECT_EQ(snappedPoint(1.125, { first, second }), first);
    EXPECT_EQ(snappedPoint(1.0625, { second }), first);
}

TEST_F(SnapperTest, interiorSnapsPreferPreviouslySnappedCoordinates)
{
    const auto snappedPoint = [](const Coordinates& coordinates, CoordinateId target) {
        Mesh mesh;
        mesh.grid = Grid{
            std::vector<double>{ 0.0, 1.0 },
            std::vector<double>{ 0.0, 1.0 },
            std::vector<double>{ 0.0, 1.0 }
        };
        mesh.coordinates = coordinates;
        mesh.groups = { Group() };
        for (CoordinateId i = 0; i < coordinates.size(); ++i) {
            mesh.groups[0].elements.emplace_back(
                std::vector<CoordinateId>{ i }, Element::Type::Node);
        }

        const Mesh result = Snapper(mesh).getMesh();
        return result.coordinates[result.groups[0].elements[target].vertices[0]];
    };

    const Coordinate midpoint({ 0.5, 0.5, 0.5 });
    const Coordinate nearUpperCorner({ 0.875, 0.875, 0.875 });
    const Coordinate lowerCorner({ 0.0, 0.0, 0.0 });
    const Coordinate upperCorner({ 1.0, 1.0, 1.0 });
    EXPECT_EQ(snappedPoint({ nearUpperCorner, midpoint }, 1), upperCorner);
    EXPECT_EQ(snappedPoint({ midpoint, nearUpperCorner }, 0), lowerCorner);
}

TEST_F(SnapperTest, snapsAcrossSharedCellEdgeConsistently)
{
    Mesh mesh;
    mesh.grid = Grid{
        std::vector<double>{ 0.0, 1.0, 2.0 },
        std::vector<double>{ 0.0, 1.0 },
        std::vector<double>{ 0.0, 1.0 }
    };
    mesh.coordinates = {
        Relative({ 1.0, 0.05, 0.0 }), // On the edge shared by cells 0 and 1.
        Relative({ 0.8, 0.4, 0.0 }), // Inside the left cell.
        Relative({ 1.0, 0.9, 0.0 }), // On the edge shared by cells 0 and 1.
        Relative({ 0.2, 0.4, 0.0 })  // Inside the left cell.
    };
    mesh.groups = { Group("surface", {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({1, 3}, Element::Type::Line)
    }) };

    SnapperOptions options;
    options.edgePoints = 3;
    options.forbiddenLength = 0.0;

    const Mesh result = Snapper(mesh, options).getMesh();

    const auto hasCoordinateNear = [&](const Coordinate& expected) {
        return std::any_of(
            result.coordinates.begin(), result.coordinates.end(),
            [&](const Coordinate& coordinate) {
                return (coordinate - expected).norm() < 1e-9;
            });
    };

    EXPECT_TRUE(hasCoordinateNear(Coordinate({ 1.0, 0.0, 0.0 })));
    EXPECT_FALSE(hasCoordinateNear(Coordinate({ 1.0, 0.5, 0.0 })));
}

TEST_F(SnapperTest, rejectsSnapsThatInvertIncidentTriangles)
{
    Mesh mesh;
    mesh.grid = buildUnitLengthGrid(1.0);
    mesh.coordinates = {
        Relative({0.328, 0.499, 0.0}),
        Relative({0.206, 0.935, 0.0}),
        Relative({0.355, 0.190, 0.0}),
        Relative({0.466, 0.718, 0.0}),
    };
    mesh.groups = {Group("surface", {
        Element({0, 1, 2}),
        Element({1, 0, 3}),
    })};
    SnapperOptions options;
    options.edgePoints = 4;
    options.forbiddenLength = 0.25;

    const Snapper snapper(mesh, options);
    const Mesh result = snapper.getMesh();

    ASSERT_EQ(result.groups.size(), 1);
    ASSERT_EQ(result.groups[0].elements.size(), 2);
    for (ElementId elementId = 0; elementId < 2; ++elementId) {
        const Coordinate originalNormal = Geometry::normal(
            Geometry::asTriV(mesh.groups[0].elements[elementId], mesh.coordinates));
        const Coordinate resultNormal = Geometry::normal(
            Geometry::asTriV(
                result.groups[0].elements[elementId], result.coordinates));
        EXPECT_GT(originalNormal * resultNormal, 0.0);
    }
    EXPECT_NO_THROW(meshTools::checkNoOverlaps(result));
    EXPECT_FALSE(hasInvalidSurfaceAdjacency(result));
    EXPECT_EQ(
        snapper.getCellsToStaircase(),
        std::set<Cell>({Cell({0, 0, 0})}));
}

TEST_F(SnapperTest, acceptsForbiddenLengthBoundaryValues)
{
    for (const double forbiddenLength : {0.0, 0.5}) {
        SnapperOptions options;
        options.forbiddenLength = forbiddenLength;
        EXPECT_NO_THROW(Snapper(buildPlane45Mesh(1.0), options));
    }
}

TEST_F(SnapperTest, rejectsInvalidForbiddenLength)
{
    for (const double forbiddenLength : {-0.1, 0.6}) {
        SnapperOptions options;
        options.forbiddenLength = forbiddenLength;
        EXPECT_THROW(Snapper(buildPlane45Mesh(1.0), options), std::logic_error);
    }
}

TEST_F(SnapperTest, snap_hole_intersections)
{
    auto m = vtkIO::readInputMesh("testData/cases/rectangular_hole/hole_1.stl");
    m.grid[X] = utils::GridTools::linspace(0.0, 200.0, 20); 
    m.grid[Y] = utils::GridTools::linspace(0.0, 110.0, 11); 
    m.grid[Z] = utils::GridTools::linspace(-100.0, 100.0, 20); 

    auto slicedMesh = Slicer{m}.getMesh();
    
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh}.getMesh();

    SnapperOptions snapperOpts;
    snapperOpts.edgePoints = 4;
    snapperOpts.forbiddenLength = 0.1;
    auto snappedMesh = Snapper{smoothedMesh}.getMesh();

    // EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    // EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
    // EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));
    // EXPECT_TRUE(meshTools::isAClosedTopology(snappedMesh.groups[0].elements));

    // //For debugging.
	meshTools::convertToAbsoluteCoordinates(snappedMesh);
	vtkIO::exportMeshToVTU("testData/cases/rectangular_hole/hole.snapper.vtk", snappedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}


// #if APP_LOADED

// TEST_F(SnapperTest, preserves_topological_closedness_for_sphere)
// {
//     auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
//     for (auto x: {X,Y,Z}) {
//         m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
//     }

//     auto slicedMesh = Slicer{m}.getMesh();
    
// 	SmootherOptions smootherOpts;
//     smootherOpts.featureDetectionAngle = 30;
//     smootherOpts.contourAlignmentAngle = 0;
// 	auto smoothedMesh = Smoother{slicedMesh}.getMesh();

//     SnapperOptions snapperOpts;
//     snapperOpts.edgePoints = 3;
//     snapperOpts.forbiddenLength = 0.3;
//     auto snappedMesh = Snapper{smoothedMesh}.getMesh();

//     EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
//     EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
//     EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));
//     EXPECT_TRUE(meshTools::isAClosedTopology(snappedMesh.groups[0].elements));

//     // //For debugging.
// 	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
// 	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);

// 	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
// 	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
// }

// # endif

// TEST_F(SnapperTest, triangles_convert_to_lines)
// {
//     SnapperOptions opts;
//     opts.edgePoints = 0;
//     opts.forbiddenLength = 0.5;
   
//     Mesh m;
//     m.grid = buildUnitLengthGrid(1.0);
//     m.coordinates = {
//         Relative({0.0, 0.0, 0.0}),
//         Relative({0.5, 0.0, 0.0}),
//         Relative({1.0, 0.0, 0.0})
//     };
//     m.groups = { Group() };
//     m.groups[0].elements = {
//         Element({0, 1, 2}),
//     };
                
//     auto res = Snapper(m, opts).getMesh();
    
//     Relatives expectedCoords = {Relative({0.0, 0.0, 0.0}), Relative({1.0, 0.0, 0.0})};
//     Element expectedElement({0, 1}, Element::Type::Line);    
    
//     EXPECT_EQ(expectedCoords, res.coordinates);
//     ASSERT_EQ(1, res.groups.size());
//     ASSERT_EQ(1, res.groups[0].elements.size());
//     EXPECT_EQ(expectedElement, res.groups[0].elements[0]);
// }

// TEST_F(SnapperTest, triangles_convert_to_nodes)
// {
//     SnapperOptions opts;
//     opts.edgePoints = 0;
//     opts.forbiddenLength = 0.5;
   
//     Mesh m;
//     m.grid = buildUnitLengthGrid(1.0);
//     m.coordinates = {
//         Relative({0.0, 0.0, 0.0}),
//         Relative({0.1, 0.0, 0.0}),
//         Relative({0.2, 0.0, 0.0})
//     };
//     m.groups = { Group() };
//     m.groups[0].elements = {
//         Element({0, 1, 2}),
//     };
                
//     auto res = Snapper(m, opts).getMesh();
    
//     Relatives expectedCoords = {Relative({0.0, 0.0, 0.0})};
//     Element expectedElement({0}, Element::Type::Node);    
    
//     EXPECT_EQ(expectedCoords, res.coordinates);
//     ASSERT_EQ(1, res.groups.size());
//     ASSERT_EQ(1, res.groups[0].elements.size());
//     EXPECT_EQ(expectedElement, res.groups[0].elements[0]);
// }
