#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "GridTools.h"

namespace meshlib::utils {

class GridToolsTest : public ::testing::Test {
public:
	const std::size_t X = 0;
	const std::size_t Y = 1;
	const std::size_t Z = 2;

	std::size_t countDifferent(const Coordinates& cs)
	{
		return std::set<Coordinate>(cs.begin(), cs.end()).size();
	}
};

TEST_F(GridToolsTest, getIntersectionsWithPlanes_1)
{
	
	TriV tri = {
		Coordinate({0.5, 0.5, 0.1}),
		Coordinate({1.5, 0.5, 0.1}),
		Coordinate({0.5, 1.5, 0.1})
	};

	GridTools gT{ utils::GridTools::buildCartesianGrid(0.0, 2.0, 3) };
	auto intL{ gT.getEdgeIntersectionsWithPlanes(tri) };
	
	ASSERT_EQ(2, intL.size());
	EXPECT_EQ(Plane(1, X), intL[0].first);
	EXPECT_EQ(LinV({ Coordinate({1.0, 1.0, 0.1}), Coordinate({1.0, 0.5, 0.1}) }), intL[0].second);
	EXPECT_EQ(Plane(1, Y), intL[1].first);
	EXPECT_EQ(LinV({ Coordinate({0.5, 1.0, 0.1}), Coordinate({1.0, 1.0, 0.1}) }), intL[1].second);
}

TEST_F(GridToolsTest, getIntersectionsWithPlanes_2)
{
	
	TriV tri = {
		Coordinate({0.0, 0.0, 0.1}),
		Coordinate({2.0, 0.0, 0.1}),
		Coordinate({0.0, 2.0, 0.1})
	};

	GridTools gT{ utils::GridTools::buildCartesianGrid(0.0, 2.0, 3) };
	auto intL{ gT.getEdgeIntersectionsWithPlanes(tri) };
	
	ASSERT_EQ(2, intL.size());
	EXPECT_EQ(Plane(1, X), intL[0].first);
	EXPECT_EQ(LinV({ Coordinate({1.0, 1.0, 0.1}), Coordinate({1.0, 0.0, 0.1}) }), intL[0].second);
	EXPECT_EQ(Plane(1, Y), intL[1].first);
	EXPECT_EQ(LinV({ Coordinate({0.0, 1.0, 0.1}), Coordinate({1.0, 1.0, 0.1}) }), intL[1].second);
}

TEST_F(GridToolsTest, getIntersectionsWithPlanes_3)
{
	
	TriV tri = {
		Coordinate({3.0, 0.0, 0.5}),
		Coordinate({0.0, 3.0, 1.0}),
		Coordinate({0.0, 3.0, 0.0})
	};

	GridTools gT{ utils::GridTools::buildCartesianGrid(0.0, 2.0, 3) };
	
	for (std::size_t d{ 0 }; d < 3; d++) {
		for (auto& c : tri) {
			std::vector<double> v{ c[0], c[1], c[2] };
			std::rotate(v.begin(), v.begin() + 1, v.end());
			c(0) = v[0]; c(1) = v[1]; c(2) = v[2];
		}
		auto intL{ gT.getEdgeIntersectionsWithPlanes(tri) };
		EXPECT_EQ(2, intL.size());
	}
}

TEST_F(GridToolsTest, getIntersectionsWithPlanes_4)
{
	TriV tri = {
		Coordinate({-28.440772135315456, -9.0000000000000000, 7.9312643478546878}),
		Coordinate({-27.292743210773551, -9.0000000000000000, 7.6236516578608731}),
		Coordinate({-21.172722176878725,  2.6462039999999991, 5.9837967401421599})
	};
	
	GridTools gT{ meshFixtures::buildProblematicTriMesh().grid };
	auto intL{ gT.getEdgeIntersectionsWithPlanes(tri) };

	EXPECT_EQ(20, intL.size());
	for (const auto& intersection : intL) {
		const auto& plane{ intersection.first };
		const auto& cell{ plane.first };
		const auto& axis{ plane.second };

		EXPECT_EQ(2, intersection.second.size());
		for (const auto& v : intersection.second) {
			EXPECT_EQ(cell, gT.getCell(v)(axis));
			EXPECT_EQ(cell, gT.getCellDir(v(axis), axis));
		}
	}
}

TEST_F(GridToolsTest, elementCrossesGrid)
{
	Coordinates cs = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({0.0, 1.0, 0.0}),
		Coordinate({2.0, 2.0, 0.0})

	};

	GridTools gT = GridTools(GridTools::buildCartesianGrid(0.0, 2.0, 3));
	EXPECT_FALSE(gT.elementCrossesGrid(Element({ 0, 1, 2 }, Element::Type::Surface), cs));
	EXPECT_TRUE(gT.elementCrossesGrid(Element({ 0, 1, 3 }, Element::Type::Surface), cs));

}

TEST_F(GridToolsTest, coordinateCellProperties_1) 
{
	Relative r({ 7.0000000000000000, 12.000000000000002, 6.6666666666666670 });

	EXPECT_FALSE(GridTools::isRelativeInterior(r));
	EXPECT_FALSE(GridTools::isRelativeInCellFace(r));
	EXPECT_TRUE( GridTools::isRelativeInCellEdge(r));
	EXPECT_FALSE(GridTools::isRelativeInCellCorner(r));
}

TEST_F(GridToolsTest, coordinateCellProperties_2) 
{
	Relative r({ 6.9999999999999982, 12.000000000000000, 6.6666666666666670 });

	EXPECT_FALSE(GridTools::isRelativeInterior(r));
	EXPECT_FALSE(GridTools::isRelativeInCellFace(r));
	EXPECT_TRUE( GridTools::isRelativeInCellEdge(r));
	EXPECT_FALSE(GridTools::isRelativeInCellCorner(r));
}

TEST_F(GridToolsTest, cellBounds)
{
	Relative r({ 1.0, 1.0, 1.0 });

	EXPECT_TRUE(GridTools::isRelativeAtCellBound(r, {1,1,1}, {X,L}));
	EXPECT_TRUE(GridTools::isRelativeAtCellBound(r, {0,0,0}, {X,U}));

	EXPECT_FALSE(GridTools::isRelativeAtCellBound(r, {1,1,1}, {X,U}));
	EXPECT_FALSE(GridTools::isRelativeAtCellBound(r, {0,0,0}, {X,L}));
}

TEST_F(GridToolsTest, getCellEdgeAxis) 
{
	{
		Relative r({ 1.0, 1.0, 2.5 });
		EXPECT_FALSE(GridTools::isRelativeInterior(r));
		EXPECT_FALSE(GridTools::isRelativeInCellFace(r));
		EXPECT_TRUE( GridTools::isRelativeInCellEdge(r));
		EXPECT_FALSE(GridTools::isRelativeInCellCorner(r));

		EXPECT_EQ(std::make_pair(true, Axis(2)), GridTools::getCellEdgeAxis(r));
	}
	{
		Relative r({ 2.5, 1.0, 1.0});
		EXPECT_FALSE(GridTools::isRelativeInterior(r));
		EXPECT_FALSE(GridTools::isRelativeInCellFace(r));
		EXPECT_TRUE( GridTools::isRelativeInCellEdge(r));
		EXPECT_FALSE(GridTools::isRelativeInCellCorner(r));

		EXPECT_EQ(std::make_pair(true, Axis(0)), GridTools::getCellEdgeAxis(r));
	}
}

TEST_F(GridToolsTest, getTouchingCells) {
	std::vector<double> pos({ 0.0, 1.0, 2.0 });
	Grid grid({pos, pos, pos});
	GridTools gT(grid);

	// Center of cell
	{
		auto cells = gT.getTouchingCells(Relative({ 1.5, 1.5, 1.5 }));
		EXPECT_EQ(1, cells.size());
		EXPECT_EQ(1, cells.count(Cell({ 1,1,1 })));
	}

	// Face of cell
	{
		auto cells = gT.getTouchingCells(Relative({ 1.0, 1.5, 1.5 }));
		EXPECT_EQ(2, cells.size());
		EXPECT_EQ(1, cells.count(Cell({ 0,1,1 })));
		EXPECT_EQ(1, cells.count(Cell({ 1,1,1})));
	}
	{
		auto cells = gT.getTouchingCells(Relative({ 0.0, 1.5, 1.5 }));
		EXPECT_EQ(1, cells.size());
		EXPECT_EQ(1, cells.count(Cell({ 0,1,1 })));
	}
	{
		auto cells = gT.getTouchingCells(Relative({ 2.0, 1.5, 1.5 }));
		EXPECT_EQ(1, cells.size());
		EXPECT_EQ(1, cells.count(Cell({ 1,1,1 })));
	}

	// Corner of cell
	{
		auto cells = gT.getTouchingCells(Relative({ 1.0, 1.0, 1.0 }));
		EXPECT_EQ(8, cells.size());
	}
	{
		auto cells = gT.getTouchingCells(Relative({ 0.0, 0.0, 0.0 }));
		EXPECT_EQ(1, cells.size());
		EXPECT_EQ(1, cells.count(Cell({0,0,0})) );
	}
	{
		auto cells = gT.getTouchingCells(Relative({ 2.0, 2.0, 2.0 }));
		EXPECT_EQ(1, cells.size());
		EXPECT_EQ(1, cells.count(Cell({ 1,1,1 })));
	}

}

TEST_F(GridToolsTest, commonPropertiesBetweenCoordinates) {
	Grid grid = GridTools::buildCartesianGrid(0, 5, 6);
	GridTools tools = GridTools(grid);

	// Interior in different cells
	{
		Relative r1 = { 0.25, 0.30, 0.65 };
		Relative r2 = { 1.25, 2.30, 3.65 };
		EXPECT_TRUE(tools.sameCellProperties(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	// Interior in same cell
	{
		Relative r1 = { 0.25, 1.30, 2.65 };
		Relative r2 = { 0.35, 1.50, 2.10 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	// One in Face and another is interior
	{
		Relative r1 = { 2.00, 1.30, 2.65 };
		Relative r2 = { 2.35, 1.50, 2.10 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r2, r1));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnFace(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnEdge(r2, r1));
	}

	// One in Edge and another is interior
	{
		Relative r1 = { 2.00, 1.00, 2.65 };
		Relative r2 = { 2.35, 1.50, 2.10 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r2, r1));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnFace(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnEdge(r2, r1));
	}

	// One in Corner and another is interior
	{
		Relative r1 = { 2.00, 1.00, 2.00 };
		Relative r2 = { 2.35, 1.50, 2.10 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r2, r1));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnFace(r2, r1));
		EXPECT_FALSE(tools.isSegmentOnEdge(r2, r1));
	}

	// One in Edge and another in Face in different cells
	{
		Relative r1 = { 2.00, 1.00, 0.30 };
		Relative r2 = { 2.35, 1.50, 2.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	// One in Corner and another in Face in different cells
	{
		Relative r1 = { 2.00, 1.00, 0.00 };
		Relative r2 = { 2.35, 1.50, 2.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	// One in Corner and another in Edge in different cells
	{
		Relative r1 = { 2.00, 1.00, 0.00 };
		Relative r2 = { 2.35, 3.00, 2.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y
	//  *-----------------*-----------------*-----------------*
	//  |                 |                 |                 |
	//  |                 |                 |       2*        |
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  *-----------------*-----------------*-----------------*
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  |                 |                 |                 |
	//  |     1*          |                 |                 |
	//  |                 |                 |                 |
	//  *-----------------*-----------------*-----------------* x
	//  
	//  
	// Two points on faces in separate cells
	{
		Relative r1 = { 0.40, 0.25, 0.00 };
		Relative r2 = { 2.50, 1.80, 0.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 2    |    |          *2     |
	//  |                 |    |    |                 |
	//  1                 |    |    |       *1        |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    *-----------------* x
	//                         |
	//                         
	// Two points on opposite faces
	{
		Relative r1 = { 1.00, 1.25, 1.60 };
		Relative r2 = { 2.00, 1.80, 1.25 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  2                 |    |    2                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |       *1        |
	//  |                 |    |    |                 |
	//  *--------1--------* x  |    *-----------------* x
	//                         |
	//                         
	// Two points on faces in different axis
	{
		Relative r1 = { 1.50, 1.00, 1.25 };
		Relative r2 = { 1.00, 1.70, 1.60 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |     *2          |
	//  |     *1,2        |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |     *1          |
	//  *-----------------* x  |    *-----------------* x
	//                         |
	//
	//  Two interior points with segment parallel to edge
	{
		Relative r1 = { 1.35, 1.45, 1.20 };
		Relative r2 = { 1.35, 1.45, 1.60 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  2                 |    |    2        *1       |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *--------1--------* x  |    *-----------------* x
	//                         |
	//                         
	// Two points with the same Z value, on faces in different axis
	{
		Relative r1 = { 1.50, 1.00, 1.60 };
		Relative r2 = { 1.00, 1.70, 1.60 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *---1-----------2-*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |   *1            |
	//  |                 |    |    |                 |
	//  |                 |    |    |               *2|
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    *-----------------* x
	//                         |
	//                         
	// Two points in the same face
	{
		Relative r1 = { 1.25, 2.00, 1.70 };
		Relative r2 = { 1.90, 2.00, 1.50 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 2
	//  |                1,2   |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 1
	//  *-----------------* x  |    *-----------------* x
	//                         |
	//
	//  Two face points with segment parallel to edge
	{
		Relative r1 = { 2.00, 1.45, 1.20 };
		Relative r2 = { 2.00, 1.45, 1.60 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	// Two edges in different cells
	{
		Relative r1 = { 2.00, 1.00, 0.50 };
		Relative r2 = { 3.00, 4.25, 2.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |       *2        |    |    |                 |
	//  |                 |    |    1                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------* x  |    *-------2---------* x
	//                         |
	//

	// Edge and face points on the same cell
	{
		Relative r1 = { 1.00, 1.00, 1.45 };
		Relative r2 = { 1.40, 1.60, 1.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |       *2        |
	//  |                 |    |    1                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-------2---------* x  |    *-----------------* x
	//                         |
	//
	// Edge and face points with shared Face
	{
		Relative r1 = { 1.00, 1.00, 1.45 };
		Relative r2 = { 1.40, 1.00, 1.60 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *-------2---------* x  |    1-------2---------* x
	//                         |
	//
	//  Two Edge Points with shared face but different edge axis
	{
		Relative r1 = { 1.00, 1.45, 1.00 };
		Relative r2 = { 1.40, 1.00, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  1                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    1-----------------2 x
	//                         |
	//
	//  Two Edge Points with one shared face and one opposite
	{
		Relative r1 = { 1.00, 1.45, 1.00 };
		Relative r2 = { 2.00, 1.80, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------*    |    *-----------------*  
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 1    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    *----------------1,2 x
	//                         |
	//
	//  Two Edge Points in subsequent edges
	{
		Relative r1 = { 2.00, 1.45, 1.00 };
		Relative r2 = { 2.00, 2.80, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 1    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    *----------------1,2 x
	//                         |
	//
	//  Two Edge Points in the same edge
	{
		Relative r1 = { 2.00, 1.45, 1.00 };
		Relative r2 = { 2.00, 1.80, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_TRUE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 2
	//  |                 |    |    |                 |
	//  1-----------------* x  |    1-----------------* x
	//                         |
	//
	//  1 Corner point and face point in opposite face
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 2.00, 1.80, 1.25 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  1-----------------*    |    1-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  *-----------------* x  |    *-----------------2 x
	//                         |
	//
	//  1 Corner point and edge point in opposite face
	{
		Relative r1 = { 1.00, 2.00, 2.00 };
		Relative r2 = { 2.00, 1.75, 1.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------2    |    *-----------------2
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------* x  |    1-----------------* x
	//                         |
	//
	//  2 opposite corner points
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 2.00, 2.00, 2.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_FALSE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |        2        |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1---------2-------* x  |    1-----------------* x
	//                         |
	//
	//  1 Corner point and 1 Face point with common axis.
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 1.60, 1.00, 1.70 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}


	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 2    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------* x  |    1-----------------2 x
	//                         |
	//
	//  1 Corner point and 1 Edge point with shared face.
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 2.00, 1.75, 1.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}

	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  2                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------* x  |   1,2----------------* x
	//                         |
	//
	//  1 Corner point and edge point with shared edge.
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 1.00, 1.70, 1.00 };

		EXPECT_FALSE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_TRUE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnEdge(r1, r2));
	}


	//  
	//  y					   |    z
	//  *-----------------2    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------* x  |    1-----------------2 x
	//                         |
	//
	//  2 Corner point with a shared face.
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 2.00, 2.00, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_FALSE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnFace(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnEdge(r1, r2));
	}


	//  
	//  y					   |    z
	//  *-----------------*    |    *-----------------*
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  |                 |    |    |                 |
	//  1-----------------2 x  |    1-----------------2 x
	//                         |
	//
	//  2 Corner point with a shared edge.
	{
		Relative r1 = { 1.00, 1.00, 1.00 };
		Relative r2 = { 2.00, 1.00, 1.00 };

		EXPECT_TRUE(tools.sameCellProperties(r1, r2));

		EXPECT_TRUE(GridTools::areCoordOnSameFace(r1, r2));
		EXPECT_TRUE(GridTools::areCoordOnSameEdge(r1, r2));
		EXPECT_FALSE(tools.isSegmentOnFace(r1, r2));
		EXPECT_TRUE(tools.isSegmentOnEdge(r1, r2));
	}
}

TEST_F(GridToolsTest, uniformDualGrid) {

	Grid grid;
	grid[0] = { 0.0, 1.0 };
	grid[1] = { 0.0, 1.0 };
	grid[2] = { 0.0, 1.0 };
	utils::GridTools gT = utils::GridTools(grid);
	Grid dualGrid;
	EXPECT_NO_THROW(dualGrid = gT.getExtendedDualGrid());
	//EXPECT_EQ(utils::GridTools(dualGrid).numCells(), gT.numCells());
	EXPECT_EQ(dualGrid[0], std::vector<CoordinateDir>({ -0.5, 0.5, 1.5 }));
	EXPECT_EQ(dualGrid[1], std::vector<CoordinateDir>({ -0.5, 0.5, 1.5 }));
	EXPECT_EQ(dualGrid[2], std::vector<CoordinateDir>({ -0.5, 0.5, 1.5 }));

}

TEST_F(GridToolsTest, nonUniformDualGrid) {

	Grid grid;
	grid[0] = { 0.0, 1.0, 1.75 };
	grid[1] = { 0.0, 1.0, 1.25};
	grid[2] = { 0.0, 1.0, 2.25 };
	utils::GridTools gT = utils::GridTools(grid);
	Grid dualGrid;
	EXPECT_NO_THROW(dualGrid = gT.getExtendedDualGrid());
	//EXPECT_EQ(utils::GridTools(dualGrid).numCells(), gT.numCells());
	EXPECT_EQ(dualGrid[0], std::vector<CoordinateDir>({ -0.5, 0.5, 1 + 0.75 / 2, 1.75 + 0.75/2 }));
	EXPECT_EQ(dualGrid[1], std::vector<CoordinateDir>({ -0.5, 0.5, 1. + 0.25 / 2, 1.25 + 0.25/2 }));
	EXPECT_EQ(dualGrid[2], std::vector<CoordinateDir>({ -0.5, 0.5, 1 + 1.25 / 2, 2.25 + 1.25/2 }));

}

}
