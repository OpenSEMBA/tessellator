#include "gtest/gtest.h"

#include "utils/ConvexHull.h"

namespace meshlib::utils {

class ConvexHullTest_meshlib : public ::testing::Test {
public:
	static Coordinates buildCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),   // Boundary point
			Coordinate({1.0, 0.0, 0.0}),   // Boundary point
			Coordinate({1.0, 1.0, 0.0}),   // Boundary point
			Coordinate({0.0, 1.0, 0.0}),   // Boundary point
			Coordinate({0.25, 0.75, 0.0}), // Inner point
			Coordinate({0.75, 0.75, 0.0}), // Inner point
			Coordinate({5.0, 5.0, 5.0}),   // Out of plane
		};
	}
};


TEST_F(ConvexHullTest_meshlib, single_point)
{
	auto coords{ buildCoordinates() };
	
	auto hull{ ConvexHull(&coords).get({ 0 }) };

	EXPECT_EQ(CoordinateIds({ 0 }), hull);
}
//
TEST_F(ConvexHullTest_meshlib, two_points)
{
	auto coords{ buildCoordinates() };

	auto hull{ ConvexHull(&coords).get({ 0, 1 }) };

	EXPECT_EQ(CoordinateIds({ 0, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, three_points)
{
	auto coords{ buildCoordinates() };

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2 }) };

	EXPECT_EQ(CoordinateIds({ 0, 2, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, in_plane_points)
{
	auto coords{ buildCoordinates() };

	auto hull{ ConvexHull(&coords).get({ 0, 1, 2, 3, 4, 5 }) };

	EXPECT_EQ(CoordinateIds({ 0, 3, 2, 1 }), hull);
}

TEST_F(ConvexHullTest_meshlib, out_of_plane)
{
	auto coords{ buildCoordinates() };

	EXPECT_ANY_THROW(
		ConvexHull(&coords).get({ 0, 1, 2, 3, 6 })
	);
}


}