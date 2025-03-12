#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "core/Slicer.h"
#include "app/vtkIO.h"

namespace meshlib::core {
using namespace utils;
using namespace meshFixtures;
using namespace meshTools;

class SmootherTest : public ::testing::Test {
protected:
	const double sSAngle = 30.0;
	const double alignmentAngle = 5.0;
};

TEST_F(SmootherTest, self_intersecting_after_smoothing)
{
	// 2
	// | \
	// |  {3,4}
	// | / \
	// 0 -- 1
	
	Mesh m;
	{
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.00, 0.00, 0.000}),
			Coordinate({1.00, 0.00, 0.000}),
			Coordinate({0.00, 1.00, 0.000}),
			Coordinate({0.50, 0.50, 0.000}),
			Coordinate({0.50, 0.50, 0.001}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 3, 1}),
			Element({3, 0, 2}),
			Element({0, 1, 4}),
			Element({0, 4, 2})
		};
	}

	auto r{ Smoother{m}.getMesh() }; 

	EXPECT_EQ(2, countMeshElementsIf(r, isTriangle));
}

TEST_F(SmootherTest, non_manifold)
{
	EXPECT_EQ(3, countMeshElementsIf(Smoother{ buildNonManifoldPatchMesh(1.0) }.getMesh(), isTriangle));
}

TEST_F(SmootherTest, touching_by_single_point)
{
	Mesh m;
	{
		// Corner.
		//      4
		//     /| 
		//    3-0
		//     /| 
		//    1-2
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.50, 0.50, 0.00}),
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.50, 0.00, 0.00}),
			Coordinate({0.00, 0.50, 0.00}),
			Coordinate({0.50, 1.00, 0.00}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({0, 4, 3})
		};

	}

	auto r{ Smoother{m}.getMesh() };

	EXPECT_EQ(1, countMeshElementsIf(r, isTriangle));
}

TEST_F(SmootherTest, preserves_topological_closedness_for_alhambra)
{
	
	auto m = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	
	m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
	
	auto slicedMesh = Slicer{m}.getMesh();
	
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh}.getMesh();
	
	EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
}


TEST_F(SmootherTest, preserves_topological_closedness_for_sphere)
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

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}

}