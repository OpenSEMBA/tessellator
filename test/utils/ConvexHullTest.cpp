#include "gtest/gtest.h"

#include "utils/ConvexHull.h"

namespace meshlib::utils {

class ConvexHullTest_meshlib : public ::testing::Test {
public:
	static Coordinates buildCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),   // 0, Boundary point
			Coordinate({1.0, 0.0, 0.0}),   // 1, Boundary point
			Coordinate({1.0, 1.0, 0.0}),   // 2, Boundary point
			Coordinate({0.0, 1.0, 0.0}),   // 3, Boundary point
			Coordinate({0.25, 0.75, 0.0}), // 4, Inner point
			Coordinate({0.75, 0.75, 0.0}), // 5, Inner point
			Coordinate({5.0, 5.0, 5.0}),   // 6, Out of plane
		};
	}
};


TEST_F(ConvexHullTest_meshlib, single_point)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });
	
	auto hull{ ConvexHull(&coords).get({ 0 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0 }), hull);
}
//
TEST_F(ConvexHullTest_meshlib, two_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, three_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 2, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, in_plane_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2, 3, 4, 5 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 3, 2, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, out_of_plane)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });
	
	auto hull{ ConvexHull(&coords).get({ 0, 1, 2, 3, 6 }, zVec) };
	
	EXPECT_EQ(CoordinateIds({ 0, 3, 6, 1 }), hull);
}


}