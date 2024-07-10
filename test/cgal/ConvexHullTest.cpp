#include "gtest/gtest.h"

#include "cgal/ConvexHull.h"

using namespace meshlib;
using namespace cgal;

class ConvexHullTest : public ::testing::Test {
public:
	static Coordinates buildCoordinates()
	{
		return Coordinates {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({5.0, 5.0, 5.0}),
		};
	}

	static Coordinates buildPathCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.25, 0.75, 0.0}),
			Coordinate({0.75, 0.75, 0.0}),
			Coordinate({5.0, 5.0, 5.0}),
		};
	}
	};



TEST_F(ConvexHullTest, convexHull)
{
	auto coords = buildPathCoordinates();

	EXPECT_EQ(3, ConvexHull(&coords).get({0, 1, 2} ).size());
	EXPECT_EQ(4, ConvexHull(&coords).get({0, 1, 2, 3, 4, 5}).size());
	
	ASSERT_NO_THROW(ConvexHull(&coords).get({ 0, 1, 2, 3, 4, 5, 6 }));

}

TEST_F(ConvexHullTest, convexHull_2)
{
	Coordinates coords = {
		Coordinate({4.0, 4.0, 0.0}),
		Coordinate({3.0, 3.0, 0.0}),
		Coordinate({4.0, 3.0, 0.0}),
		Coordinate({3.0, 4.0, 0.0}),
	};

	EXPECT_EQ(4, ConvexHull(&coords).get({ 0, 1, 2, 3 }).size());
}
