#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace utils;
using namespace meshFixtures;

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

	EXPECT_EQ(2, r.countTriangles());
}

TEST_F(SmootherTest, non_manifold)
{
	EXPECT_EQ(3, Smoother{ buildNonManifoldPatchMesh(1.0) }.getMesh().countTriangles());
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

	EXPECT_EQ(2, r.countTriangles());
}