#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Snapper.h"
#include "Slicer.h"
#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "app/vtkIO.h"

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

TEST_F(SnapperTest, preserves_topological_closedness_for_sphere)
{
    auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    auto slicedMesh = Slicer{m}.getMesh();
    
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh}.getMesh();

    SnapperOptions snapperOpts;
    snapperOpts.edgePoints = 3;
    snapperOpts.forbiddenLength = 0.3;
    auto snappedMesh = Snapper{smoothedMesh}.getMesh();

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(snappedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}


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
