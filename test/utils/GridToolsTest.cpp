#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "GridTools.h"

using namespace meshlib;
using namespace utils;

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
	EXPECT_FALSE(GridTools::isRelativeOnCellFace(r));
	EXPECT_TRUE( GridTools::isRelativeOnCellEdge(r));
	EXPECT_FALSE(GridTools::isRelativeOnCellCorner(r));
}

TEST_F(GridToolsTest, coordinateCellProperties_2) 
{
	Relative r({ 6.9999999999999982, 12.000000000000000, 6.6666666666666670 });

	EXPECT_FALSE(GridTools::isRelativeInterior(r));
	EXPECT_FALSE(GridTools::isRelativeOnCellFace(r));
	EXPECT_TRUE( GridTools::isRelativeOnCellEdge(r));
	EXPECT_FALSE(GridTools::isRelativeOnCellCorner(r));
}

TEST_F(GridToolsTest, getCellEdgeAxis) 
{
	{
		Relative r({ 1.0, 1.0, 2.5 });
		EXPECT_FALSE(GridTools::isRelativeInterior(r));
		EXPECT_FALSE(GridTools::isRelativeOnCellFace(r));
		EXPECT_TRUE( GridTools::isRelativeOnCellEdge(r));
		EXPECT_FALSE(GridTools::isRelativeOnCellCorner(r));

		EXPECT_EQ(std::make_pair(true, Axis(2)), GridTools::getCellEdgeAxis(r));
	}
	{
		Relative r({ 2.5, 1.0, 1.0});
		EXPECT_FALSE(GridTools::isRelativeInterior(r));
		EXPECT_FALSE(GridTools::isRelativeOnCellFace(r));
		EXPECT_TRUE( GridTools::isRelativeOnCellEdge(r));
		EXPECT_FALSE(GridTools::isRelativeOnCellCorner(r));

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

