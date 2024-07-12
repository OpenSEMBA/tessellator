
#include "gtest/gtest.h"

#include "tessellator/Slicer.h"
#include "cgal/Manifolder.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

using namespace meshlib;
using namespace utils;
using namespace meshTools;
using namespace meshFixtures;

class MeshToolsTest : public ::testing::Test {
public:
	
};

TEST_F(MeshToolsTest, checkNoCellsAreCrossed_tris_do_cross)
{
	
	Mesh m = buildCubeSurfaceMesh(0.2);
	GridTools gT(m.grid);
	std::transform(
		m.coordinates.begin(), m.coordinates.end(),
		m.coordinates.begin(),
		[&](auto const& c)
		{
			return gT.getRelative(c);
		}
	);

	ASSERT_ANY_THROW(checkNoCellsAreCrossed(m));
}

TEST_F(MeshToolsTest, checkNoCellsAreCrossed_tris_no_cross)
{
	auto m{ tessellator::Slicer{buildCubeSurfaceMesh(0.2)}.getMesh() };

	ASSERT_NO_THROW(checkNoCellsAreCrossed(m));
}

TEST_F(MeshToolsTest, checkNoCellsAreCrossed_lines_no_cross)
{
	Mesh m;
	m.grid = GridTools::buildCartesianGrid(0.0, 2.0, 3);
	m.coordinates = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 1.0, 0.0})
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 1}, Element::Type::Line)
	};
	
	ASSERT_NO_THROW(checkNoCellsAreCrossed(m));
}

TEST_F(MeshToolsTest, checkNoCellsAreCrossed_lines_do_cross)
{
	Mesh m;
	m.grid = GridTools::buildCartesianGrid(0.0, 2.0, 3);
	m.coordinates = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({2.0, 1.0, 0.0})
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 1}, Element::Type::Line)
	};

	ASSERT_ANY_THROW(checkNoCellsAreCrossed(m));
}

TEST_F(MeshToolsTest, checkNoOverlaps_1) 
{
	ASSERT_NO_THROW(
		checkNoOverlaps(
			buildCubeSurfaceMesh(0.2)
		)
	);
}

TEST_F(MeshToolsTest, checkNoOverlaps_2)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 1.0,  0.0}),
			Coordinate({1.0, 1.0,  0.0}),
			Coordinate({1.0, 0.0,  0.0}),
			Coordinate({0.0, 0.0, -0.2})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({2, 1, 3})
		};
	}

	ASSERT_NO_THROW(checkNoOverlaps(m));
}

TEST_F(MeshToolsTest, checkNoOverlaps_3)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 1.0,  0.0}),
			Coordinate({1.0, 1.0,  0.0}),
			Coordinate({1.0, 0.0,  0.0}),
			Coordinate({0.0, 0.0,  0.2})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({2, 1, 3})
		};
	}

	ASSERT_NO_THROW(checkNoOverlaps(m));
}

TEST_F(MeshToolsTest, checkNoOverlaps_4)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.0, 0.0, 0.0})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({2, 1, 3})
		};
	}

	ASSERT_ANY_THROW(checkNoOverlaps(m));
}

TEST_F(MeshToolsTest, checkNoAreasBelowThreshold_1)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2})
		};
	}

	ASSERT_NO_THROW(checkNoNullAreasExist(m));
}

TEST_F(MeshToolsTest, checkNoAreaBelowThreshold_2)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 0.0, 0.0})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2})
		};
	}

	ASSERT_ANY_THROW(checkNoNullAreasExist(m));
}

TEST_F(MeshToolsTest, duplicateCoordinatesUsedByDifferentGroups) 
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0})
		};

		Group g;
		g.elements = {
			Element({0, 1, 2}, Element::Type::Surface)
		};
		m.groups = { g, g };
	}

	Mesh res = duplicateCoordinatesUsedByDifferentGroups(m);
	
	ASSERT_EQ(6, res.coordinates.size());
	for (std::size_t i = 0; i < 3; i++) {
		EXPECT_EQ(res.coordinates[i], res.coordinates[i + 3]);
	}
	
	ASSERT_EQ(2, res.groups.size());
	ASSERT_EQ(1, res.groups[0].elements.size());
	EXPECT_EQ(std::vector<CoordinateId>({ 0, 1, 2 }), res.groups[0].elements[0].vertices);
	ASSERT_EQ(1, res.groups[1].elements.size());
	EXPECT_EQ(std::vector<CoordinateId>({ 3, 4, 5 }), res.groups[1].elements[0].vertices);
}

TEST_F(MeshToolsTest, duplicateCoordinatesUsedByDifferentGroups_2)
{
	Mesh m = buildPlane45TwoMaterialsMesh(1.0);

	Mesh r = duplicateCoordinatesUsedByDifferentGroups(m);

	EXPECT_NE(m.coordinates.size(), r.coordinates.size());
	EXPECT_EQ(6, m.coordinates.size());
	EXPECT_EQ(8, r.coordinates.size());
}

TEST_F(MeshToolsTest, duplicateCoordinatesUsedByDifferentGroups_sameGroup)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0})
		};

		Group g;
		g.elements = {
			Element({0, 1, 2}, Element::Type::Surface),
			Element({2, 1, 3}, Element::Type::Surface)
		};
		m.groups = { g };
	}

	Mesh res = duplicateCoordinatesUsedByDifferentGroups(m);

	EXPECT_EQ(res, m);
}

TEST_F(MeshToolsTest, getBoundingBox)
{
	Mesh m = buildTriOutOfGridMesh();

	auto bb = getBoundingBox(m);
	
	EXPECT_EQ(m.coordinates[2], bb.first);
	EXPECT_EQ(m.coordinates[0], bb.second);
}

TEST_F(MeshToolsTest, getBoundingBox_2)
{
	Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
	auto bb = getBoundingBox(m);

	EXPECT_EQ(VecD({-10.00, 0.01, 0.5 }), bb.first);
	EXPECT_EQ(VecD({ 1.99, 1.99, 0.5 }), bb.second);
}

TEST_F(MeshToolsTest, getBoundingBox_epsilon_coord)
{
	Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
	m.coordinates[2] = 
		Coordinate( { -std::numeric_limits<double>::epsilon(), 1.00, 0.5 });
	
	auto bb = getBoundingBox(m);

	EXPECT_EQ(VecD({ 0.00, 0.01, 0.5 }), bb.first);
	EXPECT_EQ(VecD({ 1.99, 1.99, 0.5}), bb.second);
}

TEST_F(MeshToolsTest, getEnlargedGridIncludingAllElements)
{
	{
		Mesh m = buildTriPartiallyOutOfGridMesh(1.0);

		Grid g = getEnlargedGridIncludingAllElements(m);

		EXPECT_EQ(g[0].size(), m.grid[0].size() + 1);
	}

	{
		Mesh m = buildTetMesh(0.25);

		Grid g = getEnlargedGridIncludingAllElements(m);

		EXPECT_EQ(m.grid, g);
	}
}

TEST_F(MeshToolsTest, reduceGrid_tri_out_of_grid_upper)
{
	Mesh m = buildTriOutOfGridMesh();
	Grid originalGrid = m.grid;

	m.grid = getEnlargedGridIncludingAllElements(m);
	
	{
		auto sliced{ tessellator::Slicer(m).getMesh() };
		EXPECT_EQ(1, sliced.countElems());
		meshTools::reduceGrid(sliced, originalGrid);
		EXPECT_EQ(0, sliced.countElems());
	}

}

TEST_F(MeshToolsTest, reduceGrid_tri_out_of_grid_lower)
{
	Mesh m = buildTriOutOfGridMesh();
	Grid originalGrid = m.grid;
	for (auto& c : m.coordinates) {
		c = -c;
	}

	m.grid = getEnlargedGridIncludingAllElements(m);

	{
		auto sliced{ tessellator::Slicer(m).getMesh() };
		EXPECT_EQ(1, sliced.countElems());
		meshTools::reduceGrid(sliced, originalGrid);
		EXPECT_EQ(0, sliced.countElems());
	}

}

TEST_F(MeshToolsTest, reduceGrid_epsilon_coord)
{
	Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
	Grid originalGrid = m.grid;
	m.coordinates[2] = 
		Coordinate( { -std::numeric_limits<double>::epsilon(), 1.00, 0.5 });

	m.grid = getEnlargedGridIncludingAllElements(m);

	{
		auto sliced{ tessellator::Slicer(m).getMesh() };
		ASSERT_NO_THROW(meshTools::reduceGrid(sliced, originalGrid));
		EXPECT_EQ(6, sliced.countElems());
	}

}

TEST_F(MeshToolsTest, setGrid)
{
	auto m{ buildTriOutOfGridMesh() };

	EXPECT_EQ(0, setGrid(m, m.grid).countElems());
}

TEST_F(MeshToolsTest, setGrid_2)
{
	Mesh m;
	{
		m.grid[X] = { -0.5, 0.0, 1.0, 1.5 };
		m.grid[Y] = m.grid[X];
		m.grid[Z] = m.grid[X];

		m.coordinates = {
			Coordinate({1.0, 1.5, 1.0}),
			Coordinate({2.0, 1.5, 1.0}),
			Coordinate({1.0, 1.5, 2.0}),
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({2, 0, 1})
		};
	}

	Grid nG;
	nG[X] = { -0.5, 0.5, 1.5 };
	nG[Y] = nG[X];
	nG[Z] = nG[X];

	Mesh r{ setGrid(m, nG) };

	EXPECT_EQ(1, r.countElems());

	ASSERT_EQ(3, r.coordinates.size());
	EXPECT_EQ(Coordinate({ 0.5, 1.0, 0.5 }), r.coordinates[0]);
}