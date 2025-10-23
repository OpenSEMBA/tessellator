
#include "gtest/gtest.h"

#include "core/Slicer.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

namespace meshlib::utils {

using namespace meshTools;
using namespace meshFixtures;

class MeshToolsTest : public ::testing::Test {
public:
	
};



TEST_F(MeshToolsTest, countElements)
{
	float lowerCoordinateValue = -0.5;
	float upperCoordinateValue = 0.5;
	int numberOfCells = 4;
	float step = 0.25;
	assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

	Mesh inputMesh;
	inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
	inputMesh.coordinates = {
		Coordinate({-0.20, -0.20, -0.20}), // 0
		Coordinate({-0.20,  0.20, -0.20}), // 1
		Coordinate({ 0.20,  0.20, -0.20}), // 2
		Coordinate({ 0.20, -0.20, -0.20}), // 3
		Coordinate({-0.20, -0.20,  0.20}), // 4
		Coordinate({-0.20,  0.20,  0.20}), // 5
		Coordinate({ 0.20,  0.20,  0.20}), // 6
		Coordinate({ 0.20, -0.20,  0.20}), // 7
	};

	inputMesh.groups.resize(8);
	inputMesh.groups[0].elements = {
		Element({0}, Element::Type::Node),
	};
	inputMesh.groups[1].elements = {
		Element({1}, Element::Type::Node),
		Element({0, 1}, Element::Type::Line),
	};
	inputMesh.groups[2].elements = {
		Element({2}, Element::Type::Node),
		Element({1, 2}, Element::Type::Line),
		Element({0, 1, 2}, Element::Type::Surface),
	};
	inputMesh.groups[3].elements = {
		Element({3}, Element::Type::Node),
		Element({2, 3}, Element::Type::Line),
		Element({3, 0}, Element::Type::Line),
		Element({2, 3, 0}, Element::Type::Surface),
		Element({0, 1, 2, 3}, Element::Type::Surface),

	};
	inputMesh.groups[4].elements = {
		Element({4}, Element::Type::Node),
		Element({0, 4}, Element::Type::Line),
		Element({0, 1, 4}, Element::Type::Surface),
		Element({0, 1, 3, 4}, Element::Type::Volume),
	};
	inputMesh.groups[5].elements = {
		Element({5}, Element::Type::Node),
		Element({1, 5}, Element::Type::Line),
		Element({4, 5}, Element::Type::Line),
		Element({1, 5, 4}, Element::Type::Surface),
		Element({1, 2, 5}, Element::Type::Surface),
		Element({0, 1, 5, 4}, Element::Type::Surface),
	};
	inputMesh.groups[6].elements = {
		Element({6}, Element::Type::Node),
		Element({2, 6}, Element::Type::Line),
		Element({5, 6}, Element::Type::Line),
		Element({2, 6, 5}, Element::Type::Surface),
		Element({2, 3, 6}, Element::Type::Surface),
		Element({4, 5, 6}, Element::Type::Surface),
		Element({1, 2, 6, 5}, Element::Type::Surface),
		Element({1, 5, 4, 6}, Element::Type::Volume),
		Element({1, 2, 3, 6}, Element::Type::Volume),
		Element({1, 3, 4, 6}, Element::Type::Volume),
	};
	inputMesh.groups[7].elements = {
		Element({7}, Element::Type::Node),
		Element({3, 7}, Element::Type::Line),
		Element({6, 7}, Element::Type::Line),
		Element({7, 4}, Element::Type::Line),
		Element({3, 7, 6}, Element::Type::Surface),
		Element({3, 0, 7}, Element::Type::Surface),
		Element({0, 4, 7}, Element::Type::Surface),
		Element({6, 7, 4}, Element::Type::Surface),
		Element({2, 3, 7, 6}, Element::Type::Surface),
		Element({3, 0, 4, 7}, Element::Type::Surface),
		Element({4, 5, 6, 7}, Element::Type::Surface),
		Element({3, 6, 7, 4}, Element::Type::Volume),
	};
	// 43

	ASSERT_EQ(8, countMeshElementsIf(inputMesh, isNode));
	ASSERT_EQ(35, countMeshElementsIf(inputMesh, isNotNode));
	ASSERT_EQ(12, countMeshElementsIf(inputMesh, isLine));
	ASSERT_EQ(31, countMeshElementsIf(inputMesh, isNotLine));
	ASSERT_EQ(12, countMeshElementsIf(inputMesh, isTriangle));
	ASSERT_EQ(31, countMeshElementsIf(inputMesh, isNotTriangle));
	ASSERT_EQ(6, countMeshElementsIf(inputMesh, isQuad));
	ASSERT_EQ(37, countMeshElementsIf(inputMesh, isNotQuad));
	ASSERT_EQ(5, countMeshElementsIf(inputMesh, isTetrahedron));
	ASSERT_EQ(38, countMeshElementsIf(inputMesh, isNotTetrahedron));
}



TEST_F(MeshToolsTest, canGetHighestDimensionByGroup)
{
	float lowerCoordinateValue = -0.5;
	float upperCoordinateValue = 0.5;
	int numberOfCells = 4;
	float step = 0.25;
	assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

	Mesh inputMesh;
	inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
	inputMesh.coordinates = {
		Coordinate({-0.20, -0.20, -0.20}), // 0
		Coordinate({-0.20,  0.20, -0.20}), // 1
		Coordinate({ 0.20,  0.20, -0.20}), // 2
		Coordinate({ 0.20, -0.20, -0.20}), // 3
		Coordinate({-0.20, -0.20,  0.20}), // 4
		Coordinate({-0.20,  0.20,  0.20}), // 5
		Coordinate({ 0.20,  0.20,  0.20}), // 6
		Coordinate({ 0.20, -0.20,  0.20}), // 7
	};

	inputMesh.groups.resize(11);
	inputMesh.groups[0].elements = {
		Element({0}, Element::Type::Node),
	};
	inputMesh.groups[1].elements = {
		Element({1}, Element::Type::Node),
		Element({0, 1}, Element::Type::Line),
	};
	inputMesh.groups[2].elements = {
		Element({2}, Element::Type::Node),
		Element({1, 2}, Element::Type::Line),
		Element({0, 1, 2}, Element::Type::Surface),
	};
	inputMesh.groups[3].elements = {
		Element({3}, Element::Type::Node),
		Element({2, 3}, Element::Type::Line),
		Element({1, 2, 3}, Element::Type::Surface),
		Element({0, 1, 2, 3}, Element::Type::Surface),
	};
	inputMesh.groups[4].elements = {
		Element({4}, Element::Type::Node),
		Element({0, 4}, Element::Type::Line),
		Element({0, 1, 4}, Element::Type::Surface),
		Element({0, 1, 3, 4}, Element::Type::Volume),
	};
	inputMesh.groups[5].elements = {
		Element({5}, Element::Type::Node),
		Element({1, 5}, Element::Type::Line),
		Element({1, 5, 4}, Element::Type::Surface),
		Element({0, 1, 5, 4}, Element::Type::Surface),
		Element({0, 1, 3, 4}, Element::Type::Volume),
	};
	inputMesh.groups[6].elements = {
		Element({1, 3, 4, 6}, Element::Type::Volume),
	};
	inputMesh.groups[7].elements = {
		Element({3, 6, 7, 4}, Element::Type::Volume),
		Element({4, 5, 6, 7}, Element::Type::Surface),
		Element({0, 4, 7}, Element::Type::Surface),
		Element({7, 4}, Element::Type::Line),
		Element({7}, Element::Type::Node),
	};
	inputMesh.groups[8].elements = {
		Element({3, 6, 7, 4}, Element::Type::Volume),
		Element({4, 5, 6, 7}, Element::Type::Surface),
	};
	inputMesh.groups[9].elements = {
		Element({4, 5, 6, 7}, Element::Type::Surface),
		Element({0, 4, 7}, Element::Type::Surface),
	};
	// 43

	std::vector<Element::Type> expectedList({
		Element::Type::Node,
		Element::Type::Line,
		Element::Type::Surface,
		Element::Type::Surface,
		Element::Type::Volume,
		Element::Type::Volume,
		Element::Type::Volume,
		Element::Type::Volume,
		Element::Type::Volume,
		Element::Type::Surface,
		Element::Type::None,
	});

	auto highestDimensions = getHighestDimensionByGroup(inputMesh);

	ASSERT_EQ(expectedList.size(), highestDimensions.size());
	for (GroupId index = 0; index < expectedList.size(); ++index) {
		ASSERT_EQ(expectedList[index], highestDimensions[index]) << "Current Group: #" << index << std::endl;
	}
}

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
	auto m{ core::Slicer{buildCubeSurfaceMesh(0.2)}.getMesh() };

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

TEST_F(MeshToolsTest, testNodesAreValid)
{
	Mesh m;
	Coordinate expectedCoordinate;
	{
		m.coordinates = { expectedCoordinate };

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0}, Element::Type::Node),
		};
	}

	ASSERT_NO_THROW(checkNoNullAreasExist(m));

	ASSERT_EQ(m.coordinates.size(), 1);
	for (Axis axis = X; axis <= Z; ++axis) {
		EXPECT_EQ(m.coordinates[0][axis], expectedCoordinate[axis]);
	}
	ASSERT_EQ(m.groups.size(), 1);
	ASSERT_EQ(m.groups[0].elements.size(), 1);

	auto& resultElement = m.groups[0].elements[0];
	ASSERT_EQ(resultElement.vertices.size(), 1);
	ASSERT_EQ(resultElement.vertices[0], 0);
	ASSERT_TRUE(resultElement.isNode());
}

TEST_F(MeshToolsTest, testLinesAreValidUnlessLengthIsZero)
{
	Coordinates expectedCoordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.1, 0.1, 0.1})
	};
	auto expectedElement = Element({ 0, 1 }, Element::Type::Line);
	Mesh m;
	{

		m.coordinates = expectedCoordinates;

		m.groups = { Group() };
		m.groups[0].elements = { expectedElement };
	}

	ASSERT_NO_THROW(checkNoNullAreasExist(m));

	ASSERT_EQ(m.coordinates.size(), expectedCoordinates.size());
	for (std::size_t c = 0; c < expectedCoordinates.size(); ++c) {
		for (Axis axis = X; axis <= Z; ++axis) {
			EXPECT_EQ(m.coordinates[c][axis], expectedCoordinates[c][axis]);
		}
	}
	ASSERT_EQ(m.groups.size(), 1);
	ASSERT_EQ(m.groups[0].elements.size(), 1);

	auto& resultElement = m.groups[0].elements[0];

	ASSERT_TRUE(resultElement.isLine());
	ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size());
	for (std::size_t v = 0; v < expectedCoordinates.size(); ++v) {
		EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
	}

	m.coordinates.push_back(Coordinate({ 0.1, 0.1, 0.1 }));
	m.groups[0].elements.push_back(Element({ 1, 2 }, Element::Type::Line));

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

TEST_F(MeshToolsTest, duplicateCoordinatesSharedBySingleTrianglesVertex)
{
	Mesh m;
	{
		// Corner.
		//      4
		//     /| 
		//    3-0
		//     /| 
		//    1-2
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.50, 0.50, 0.00}),
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.50, 0.00, 0.00}),
			Coordinate({0.00, 0.50, 0.00}),
			Coordinate({0.50, 1.00, 0.00}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({0, 4, 3})
		};
	}

	Mesh r = duplicateCoordinatesSharedBySingleTrianglesVertex(m);

	EXPECT_EQ(6, r.coordinates.size());
	EXPECT_EQ(2, r.groups[0].elements.size());
	EXPECT_EQ(CoordinateIds({0, 1, 2}), r.groups[0].elements[0].vertices);
	EXPECT_EQ(CoordinateIds({5, 4, 3}), r.groups[0].elements[1].vertices);
}

TEST_F(MeshToolsTest, duplicateCoordinatesSharedBySingleTrianglesVertex_2)
{
	Mesh m;
	{
		// Corner.
		//      4 6
		//     /|/|
		//    3-0-5
		//     /| 
		//    1-2
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.50, 0.50, 0.00}),
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.50, 0.00, 0.00}),
			Coordinate({0.00, 0.50, 0.00}),
			Coordinate({0.50, 1.00, 0.00}),
			Coordinate({1.00, 0.50, 0.00}),
			Coordinate({1.00, 1.00, 0.00}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({0, 4, 3}),
			Element({5, 6, 0})
		};
	}

	Mesh r = duplicateCoordinatesSharedBySingleTrianglesVertex(m);

	EXPECT_EQ(9, r.coordinates.size());
	EXPECT_EQ(3, r.groups[0].elements.size());
	EXPECT_EQ(CoordinateIds({0, 1, 2}), r.groups[0].elements[0].vertices);
	EXPECT_EQ(CoordinateIds({7, 4, 3}), r.groups[0].elements[1].vertices);
	EXPECT_EQ(CoordinateIds({5, 6, 8}), r.groups[0].elements[2].vertices);
}

TEST_F(MeshToolsTest, duplicateCoordinatesSharedBySingleTrianglesVertex_3)
{
	Mesh m;
	{
		// Corner.
		//      4-6
		//     /|/|
		//    3-0-5
		//     /| 
		//    1-2
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.50, 0.50, 0.00}),
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.50, 0.00, 0.00}),
			Coordinate({0.00, 0.50, 0.00}),
			Coordinate({0.50, 1.00, 0.00}),
			Coordinate({1.00, 0.50, 0.00}),
			Coordinate({1.00, 1.00, 0.00}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({0, 4, 3}),
			Element({5, 6, 0}),
			Element({6, 4, 0}),
		};
	}

	Mesh r = duplicateCoordinatesSharedBySingleTrianglesVertex(m);

	EXPECT_EQ(8, r.coordinates.size());
	EXPECT_EQ(4, r.groups[0].elements.size());
	EXPECT_EQ(CoordinateIds({0, 1, 2}), r.groups[0].elements[0].vertices);
	EXPECT_EQ(CoordinateIds({7, 4, 3}), r.groups[0].elements[1].vertices);
	EXPECT_EQ(CoordinateIds({5, 6, 7}), r.groups[0].elements[2].vertices);
	EXPECT_EQ(CoordinateIds({6, 4, 7}), r.groups[0].elements[3].vertices);
}

TEST_F(MeshToolsTest, getElementsBoundingBox)
{
	Mesh m = buildTriOutOfGridMesh();

	auto bb = getElementsBoundingBox(m);
	
	EXPECT_EQ(m.coordinates[2], bb.first);
	EXPECT_EQ(m.coordinates[0], bb.second);
}

TEST_F(MeshToolsTest, getBoundingBox_2)
{
	Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
	auto bb = getElementsBoundingBox(m);

	EXPECT_EQ(VecD({-10.00, 0.01, 0.5 }), bb.first);
	EXPECT_EQ(VecD({ 1.99, 1.99, 0.5 }), bb.second);
}

TEST_F(MeshToolsTest, getBoundingBox_epsilon_coord)
{
	Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
	m.coordinates[2] = 
		Coordinate( { -std::numeric_limits<double>::epsilon(), 1.00, 0.5 });
	
	auto bb = getElementsBoundingBox(m);

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
	
	auto sliced{ core::Slicer(m).getMesh() };
	EXPECT_EQ(1, sliced.countElems());
	meshTools::reduceGrid(sliced, originalGrid);
	EXPECT_EQ(0, sliced.countElems());

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
		auto sliced{ core::Slicer(m).getMesh() };
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
		auto sliced{ core::Slicer(m).getMesh() };
		ASSERT_NO_THROW(meshTools::reduceGrid(sliced, originalGrid));
		EXPECT_EQ(6, sliced.countElems());
	}
}

}