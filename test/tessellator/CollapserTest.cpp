#include "gtest/gtest.h"

#include "Slicer.h"
#include "Collapser.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/CoordGraph.h"
#include "utils/GridTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace utils;

class CollapserTest : public ::testing::Test {
protected:
	
	static Mesh buildTinyTriMesh() 
	{
		Mesh m;
		m.coordinates = {
			Coordinate({0.000, 0.000, 0.000}),
			Coordinate({1.000, 0.000, 0.000}),
			Coordinate({1.000, 1.000, 0.000}),
			Coordinate({0.000, 1.000, 0.000}),
			Coordinate({0.001, 0.998, 0.000}),
			Coordinate({0.002, 0.999, 0.000}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 4}, Element::Type::Surface),
			Element({1, 5, 4}, Element::Type::Surface),
			Element({1, 2, 5}, Element::Type::Surface),
			Element({2, 3, 5}, Element::Type::Surface),
			Element({3, 4, 5}, Element::Type::Surface),
			Element({0, 4, 3}, Element::Type::Surface)
		};
		return m;
	}
	
	static Mesh buildTinyTriClosedMesh() 
	{
		Mesh m = buildTinyTriMesh();
		
		m.coordinates.push_back(Coordinate({ 0.0, 0.0, -1.0 }));
		m.coordinates.push_back(Coordinate({ 1.0, 0.0, -1.0 }));
		m.coordinates.push_back(Coordinate({ 1.0, 1.0, -1.0 }));
		m.coordinates.push_back(Coordinate({ 0.0, 1.0, -1.0 }));

		m.groups[0].elements.push_back(Element({ 0, 3, 9 }, Element::Type::Surface));
		m.groups[0].elements.push_back(Element({ 0, 9, 6 }, Element::Type::Surface));

		m.groups[0].elements.push_back(Element({ 6, 9, 8 }, Element::Type::Surface));
		m.groups[0].elements.push_back(Element({ 6, 8, 7 }, Element::Type::Surface));
		
		m.groups[0].elements.push_back(Element({ 1, 8, 2 }, Element::Type::Surface));
		m.groups[0].elements.push_back(Element({ 1, 7, 8 }, Element::Type::Surface));
		
		m.groups[0].elements.push_back(Element({ 0, 6, 7 }, Element::Type::Surface));
		m.groups[0].elements.push_back(Element({ 0, 7, 1 }, Element::Type::Surface));

		m.groups[0].elements.push_back(Element({ 3, 8, 9 }, Element::Type::Surface));
		m.groups[0].elements.push_back(Element({ 3, 2, 8 }, Element::Type::Surface));

		return m;
	}

	static Grid buildGridSize2()
	{
		Grid grid;
		std::size_t num = (std::size_t)(2.0 / 0.1) + 1;
		grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
		grid[1] = grid[0];
		grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
		return grid;
	}

	static bool isClosed(const Mesh& m) 
	{
		return CoordGraph(m.groups[0].elements)
				.getBoundaryGraph()
				.getVertices()
				.size() == 0;
	}
};

TEST_F(CollapserTest, collapser)
{
	Mesh m;
	m.coordinates = {
		Coordinate({0.000, 0.000, 0.000}),
		Coordinate({1.000, 0.000, 0.000}),
		Coordinate({1.000, 1.000, 0.000}),
		Coordinate({0.000, 1.000, 0.000}),
		Coordinate({0.000, 0.999, 0.000}),
		Coordinate({0.001, 1.000, 0.000}),
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 1, 4}, Element::Type::Surface),
		Element({1, 5, 4}, Element::Type::Surface),
		Element({1, 2, 5}, Element::Type::Surface),
		Element({3, 4, 5}, Element::Type::Surface)
	};

	Mesh r = Collapser(m, 2).getMesh();
	EXPECT_EQ(4, r.coordinates.size());
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(2, r.groups[0].elements.size());
}


TEST_F(CollapserTest, collapser_2)
{
	Mesh m = buildTinyTriMesh();

	Mesh r = Collapser(m, 2).getMesh();
	EXPECT_EQ(4, r.coordinates.size());
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(2, r.groups[0].elements.size());
}

TEST_F(CollapserTest, collapser_3)
{
	Mesh m = buildTinyTriMesh();

	Mesh r = Collapser(m, 4).getMesh();

	EXPECT_EQ(m.groups, r.groups);
	for (std::size_t i = 0; i < m.coordinates.size(); i++) {
		for (std::size_t d = 0; d < 3; d++) {
			EXPECT_NEAR(m.coordinates[i](d), r.coordinates[i](d), 1e-8);
		}
	}
}

TEST_F(CollapserTest, preserves_closedness)
{
	Mesh m = buildTinyTriClosedMesh();

	Mesh r = Collapser(m, 2).getMesh();

	EXPECT_EQ(8, r.coordinates.size());
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(12, r.groups[0].elements.size());

	EXPECT_TRUE(isClosed(m));
	EXPECT_TRUE(isClosed(r));
}

TEST_F(CollapserTest, areas_are_below_threshold_issue)
{
	Mesh m;
	m.grid = buildGridSize2();

	m.coordinates = {
		Coordinate({+2.27145875e-01, +1.01515934e-01,  +1.00000000e+00}),
		Coordinate({+1.71006665e-01, +1.80716639e-01,  +1.00000000e+00}),
		Coordinate({+9.45528424e-02, +9.62188501e-02,  +1.00000000e+00})
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 1, 2}, Element::Type::Surface)
	};

	m = Slicer{ m }.getMesh();
	
	ASSERT_NO_THROW(Collapser(m, 2));
}


TEST_F(CollapserTest, areas_are_below_threshold_issue_2)
{
	Mesh m;
	m.grid = buildGridSize2();

	m.coordinates = {
		Coordinate({ -1.71104779e-01, -1.80305297e-01, +1.00000000e+00}),
		Coordinate({ -1.79120352e-01, -2.68072696e-01, +1.00000000e+00}),
		Coordinate({ -2.68072696e-01, -1.79120352e-01, +1.00000000e+00})
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 2, 1}, Element::Type::Surface)
	};

	m = Slicer{m}.getMesh();
	ASSERT_NO_THROW(Collapser(m, 2));
}