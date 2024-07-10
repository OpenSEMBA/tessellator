#include "gtest/gtest.h"

#include "cgal/LSFPlane.h"

using namespace meshlib;
using namespace cgal;

class LSFPlaneTest : public ::testing::Test {
public:
	static Coordinates buildZPlaneCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
		};
	}

	static Coordinates buildXPlaneCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({0.0, 1.0, 1.0}),
			Coordinate({0.0, 0.0, 1.0}),
		};
	}

	static Coordinates buildYPlaneCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 1.0}),
			Coordinate({0.0, 0.0, 1.0}),
		};
	}

	static Coordinates buildYZPlaneCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({1.0, 1.0, 1.0}),
			Coordinate({0.0, 1.0, 1.0}),
		};
	}

	static Coordinates buildYZPlaneCoordinatesFlip()
	{
		return Coordinates{
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({0.0, 0.0, 1.0}),
			Coordinate({1.0, 0.0, 1.0}),
			Coordinate({1.0, 1.0, 0.0}),
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



TEST_F(LSFPlaneTest, DISABLED_fitXYZPlanes)
{
	{
		auto coords = buildXPlaneCoordinates();
		VecD normal({1,0,0});
		EXPECT_EQ(normal, LSFPlane(coords.begin(), coords.end()).getNormal());
	}
	{
		auto coords = buildYPlaneCoordinates();
		VecD normal({ 0, 1, 0 });
		EXPECT_EQ(normal, LSFPlane(coords.begin(), coords.end()).getNormal());
	}
	{
		auto coords = buildZPlaneCoordinates();
		VecD normal({ 0, 0, 1 });
		EXPECT_EQ(normal, LSFPlane(coords.begin(), coords.end()).getNormal());
	}
}

TEST_F(LSFPlaneTest, DISABLED_fitYZPlane)
{
	{
		VecD normal({ 0,-1 / sqrt(2.0),1 / sqrt(2.0) });
		auto coords = buildYZPlaneCoordinates();
		VecD planeNormal = LSFPlane(coords.begin(), coords.end()).getNormal();
		EXPECT_DOUBLE_EQ(normal[0], planeNormal[0]);
		EXPECT_DOUBLE_EQ(normal[1], planeNormal[1]);
		EXPECT_DOUBLE_EQ(normal[2], planeNormal[2]);
	}
	{
		VecD normal({ 0, 1 / sqrt(2.0),1 / sqrt(2.0) });
		auto coords = buildYZPlaneCoordinatesFlip();
		VecD planeNormal = LSFPlane(coords.begin(), coords.end()).getNormal();
		EXPECT_DOUBLE_EQ(normal[0], planeNormal[0]);
		EXPECT_DOUBLE_EQ(normal[1], planeNormal[1]);
		EXPECT_DOUBLE_EQ(normal[2], planeNormal[2]);
	}


}
