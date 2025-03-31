#include "gtest/gtest.h"

#include "utils/ConvexHull.h"

namespace meshlib::utils {

class ConvexHullTest : public ::testing::Test {
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


TEST_F(ConvexHullTest, single_point)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });
	
	auto hull{ ConvexHull(&coords).get({ 0 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0 }), hull);
}
//
TEST_F(ConvexHullTest, two_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 1 }), hull);
}

TEST_F(ConvexHullTest, three_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 2, 1 }), hull);
}

TEST_F(ConvexHullTest, in_plane_points)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2, 3, 4, 5 }, zVec) };

	EXPECT_EQ(CoordinateIds({ 0, 3, 2, 1 }), hull);
}

TEST_F(ConvexHullTest, coords_in_diag_plane)
{
	Coordinates coords{
			Coordinate({2.6938780000000002, 2.6938780000000002, 2.6938780000000002}),
			Coordinate({2.8979590000000002, 2.7959179999999999, 2.8979590000000002}),
			Coordinate({3.0,                3.0,                3.0}),  
		};
	const VecD nVec({ 300.0, 0.0, -300.0 });

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2}, nVec) };

	EXPECT_EQ(CoordinateIds({ 0, 1, 2 }), hull);
}

TEST_F(ConvexHullTest, out_of_plane)
{
	auto coords{ buildCoordinates() };
	const VecD zVec({ 0.0, 0.0, 1.0 });
	
	auto hull{ ConvexHull(&coords).get({ 0, 1, 2, 3, 6 }, zVec) };
	
	EXPECT_EQ(CoordinateIds({ 0, 3, 6, 1 }), hull);
}


}