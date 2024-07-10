#include "gtest/gtest.h"

#include "cgal/Delaunator.h"
#include "utils/Types.h"

using namespace meshlib;
using namespace cgal;

class DelaunatorTest : public ::testing::Test {
public:
	static Coordinates buildCoordinates()
	{
		return Coordinates {
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.00, 1.00, 0.00}),
			Coordinate({1.00, 1.00, 0.00}),
			Coordinate({1.00, 0.00, 0.00}),
			Coordinate({5.00, 5.00, 5.00}),
			Coordinate({0.00, 1.00, 0.00}),
			Coordinate({0.75, 0.25, 0.00})
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

TEST_F(DelaunatorTest, mesh_with_repeated_ids)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);

	{
		auto tris = delaunator.mesh({}, { {0, 1, 2, 0} });
		EXPECT_EQ(1, tris.size());
		for (auto const& tri : tris) {
			EXPECT_TRUE(tri.isTriangle());
			EXPECT_EQ(CoordinateIds({ 1, 0, 2 }), tri.vertices);
		}
	}

	{
		auto tris = delaunator.mesh({}, { {0, 1, 2, 2} });
		EXPECT_EQ(1, tris.size());
		for (auto const& tri : tris) {
			EXPECT_TRUE(tri.isTriangle());
			EXPECT_EQ(CoordinateIds({ 1, 0, 2 }), tri.vertices);
		}
	}

	{
		auto tris = delaunator.mesh({}, { {0, 1, 2, 1, 2} });
		EXPECT_EQ(1, tris.size());
		for (auto const& tri : tris) {
			EXPECT_TRUE(tri.isTriangle());
			EXPECT_EQ(CoordinateIds({ 1, 0, 2 }), tri.vertices);
		}
	}

	{
		auto tris = delaunator.mesh({}, { {0, 1, 2, 3, 6, 2} });
		ASSERT_EQ(2, tris.size());
		for (auto const& tri : tris) {
			EXPECT_TRUE(tri.isTriangle());
		}
		EXPECT_EQ(CoordinateIds({ 6, 3, 2 }), tris[0].vertices);
		EXPECT_EQ(CoordinateIds({ 0, 2, 1 }), tris[1].vertices);
	}
}

TEST_F(DelaunatorTest, mesh_one_triangle_with_constraining_polygon)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);

	auto tris = delaunator.mesh({}, { {0, 1, 2} });

	EXPECT_EQ(1, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}

}

TEST_F(DelaunatorTest, mesh_bowtie)
{
	std::vector<Coordinate> coords(5);
	coords [0] = Coordinate{{0.0, 0.0  , 0.0 }};
	coords [1] = Coordinate{{0.25, 0.25, 0.0 }};
	coords [2] = Coordinate{{0.5, 0.0  , 0.0 }};
	coords [3] = Coordinate{{0.75, 0.25, 0.0 }};
	coords [4] = Coordinate{{1.0, 0.0  , 0.0 }};


	Delaunator delaunator(&coords);
	IdSet cIds = { 0,1,2,3,4,2 };
	Delaunator::Polygon boundary = { 0,1,2,3,4,2 };
	auto tris = delaunator.mesh({}, { boundary });

	EXPECT_EQ(2, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}
TEST_F(DelaunatorTest, mesh_bowtie_2)
{
	std::vector<Coordinate> coords(6);
	coords [0] = Coordinate{{0.0, 0.0, 0.0 }};
	coords [1] = Coordinate{{1.0, 1.0, 0.0 }};
	coords [2] = Coordinate{{0.0, 1.0, 0.0 }};
	coords [3] = Coordinate{{1.0, 0.0, 0.0 }};
	coords [4] = Coordinate{{0.0, 0.25, 0.0 }};
	coords [5] = Coordinate{{0.2, 0.2, 0.0 }};


	Delaunator delaunator(&coords);
	Delaunator::Polygon boundary = { 0,5,4,1,2,4 };
	auto tris = delaunator.mesh({}, { boundary });

	EXPECT_EQ(2, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}

TEST_F(DelaunatorTest, mesh_tri_with_hole)
{
	std::vector<Coordinate> coords(6);
	coords[0] = Coordinate{ {0.0,  0.0, 0.0 } };
	coords[1] = Coordinate{ {0.3,  0.0, 0.0 } };
	coords[2] = Coordinate{ {0.15, 0.3, 0.0 } };
	coords[3] = Coordinate{ {0.1,  0.1, 0.0 } };
	coords[4] = Coordinate{ {0.2,  0.1, 0.0 } };
	coords[5] = Coordinate{ {0.15, 0.2, 0.0 } };


	Delaunator delaunator(&coords);
	Delaunator::Polygon outer_boundary = { 0,1,2};
	Delaunator::Polygon inner_boundary = { 3,4,5 };
	auto tris = delaunator.mesh({}, { outer_boundary, inner_boundary });

	EXPECT_EQ(6, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}

TEST_F(DelaunatorTest, mesh_one_triangle_other_call)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);
	IdSet cIds = { 0,1,2 };
	Delaunator::Polygon boundary = { 0,1,2 };
	auto tris = delaunator.mesh(cIds, {boundary});

	EXPECT_EQ(1, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}

TEST_F(DelaunatorTest, mesh_with_invalid_constraining_polygon)
{
	std::vector<Coordinate> coords(5);
	coords[0] = Coordinate{ {0.25, 0.00, 0.00} };
	coords[1] = Coordinate{ {0.50, 0.00, 0.00} };
	coords[2] = Coordinate{ {1.00, 0.00, 0.50} };
	coords[3] = Coordinate{ {0.25, 0.00, 1.00} };
	coords[4] = Coordinate{ {1.00, 0.00, 1.00} };

	Delaunator delaunator(&coords);
	{
		std::vector<CoordinateId> constrainingPolygon = { 0,1,2,3,4 };
		EXPECT_ANY_THROW(auto tris = delaunator.mesh({}, { constrainingPolygon }));
	}
	{
		std::vector<CoordinateId> constrainingPolygon = { 0,1,2,4,3 };
		auto tris = delaunator.mesh({}, { constrainingPolygon });
		for (const auto& tri : tris) {
			for (const auto& vertex : tri.vertices) {
				EXPECT_TRUE(find(
					constrainingPolygon.begin(),
					constrainingPolygon.end(),
					vertex) != constrainingPolygon.end());
			}
		}
	}
}
TEST_F(DelaunatorTest, mesh_two_triangles)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);

	auto tris = delaunator.mesh({}, { {0, 1, 2, 3} });

	EXPECT_EQ(2, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}

TEST_F(DelaunatorTest, mesh_example_path)
{
	auto coords = buildPathCoordinates();
	Delaunator delaunator(&coords);

	auto tris = delaunator.mesh({}, { { 0, 1, 4, 5, 2, 3 } });

	EXPECT_EQ(4, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}
}

TEST_F(DelaunatorTest, mesh_polygons_cw_or_ccw) {
	Coordinates c{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({0.1, 0.2, 0.0}),
			Coordinate({0.1, 0.9, 0.0}),
			Coordinate({0.8, 0.9, 0.0})
	};

	Delaunator delaunator(&c);
	Delaunator::Polygon p1 = { 0, 1, 2 };

	auto trisCW  = delaunator.mesh({}, { p1, { 3, 4, 5 } });
	auto trisCCW = delaunator.mesh({}, { p1, { 5, 4, 3 } });
	
	EXPECT_EQ(trisCW.size(), trisCCW.size());
}

TEST_F(DelaunatorTest, when_not_aligned)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);

	EXPECT_NO_THROW(delaunator.mesh({ 0, 1, 2, 4 }));
}

TEST_F(DelaunatorTest, throw_when_coordinateId_is_out_of_range)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);

	EXPECT_ANY_THROW(delaunator.mesh({ 0, 1, 2, 350 }));
}

TEST_F(DelaunatorTest, DISABLED_mesh_repeating_ids)
{
	auto coords = buildCoordinates();
	Delaunator delaunator(&coords);
	EXPECT_ANY_THROW(delaunator.mesh({}, { { 0, 1, 5, 2, 3 } } ));
}
