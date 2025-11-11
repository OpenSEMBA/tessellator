#include "gtest/gtest.h"

#include "Slicer.h"
#include "Collapser.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/CoordGraph.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"
#include "app/vtkIO.h"


namespace meshlib::core {

using namespace utils;
using namespace meshTools;

class CollapserTest : public ::testing::Test {
protected:
	
	static Mesh buildTinyTriMesh() 
	{
		Mesh m;
		m.grid = buildGridSize2();
		m.coordinates = {
			Relative({0.000, 0.000, 0.000}),
			Relative({1.000, 0.000, 0.000}),
			Relative({1.000, 1.000, 0.000}),
			Relative({0.000, 1.000, 0.000}),
			Relative({0.001, 0.998, 0.000}),
			Relative({0.002, 0.999, 0.000}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 4}),
			Element({1, 5, 4}),
			Element({1, 2, 5}),
			Element({2, 3, 5}),
			Element({3, 4, 5}),
			Element({0, 4, 3})
		};
		return m;
	}
	
	static Mesh buildTinyTriClosedMesh() 
	{
		Mesh m = buildTinyTriMesh();
		
		m.coordinates.push_back(Coordinate({ 0.0, 0.0, 1.0 }));
		m.coordinates.push_back(Coordinate({ 1.0, 0.0, 1.0 }));
		m.coordinates.push_back(Coordinate({ 1.0, 1.0, 1.0 }));
		m.coordinates.push_back(Coordinate({ 0.0, 1.0, 1.0 }));

		m.groups[0].elements.push_back(Element({ 0, 3, 9 }));
		m.groups[0].elements.push_back(Element({ 0, 9, 6 }));
		m.groups[0].elements.push_back(Element({ 6, 9, 8 }));
		m.groups[0].elements.push_back(Element({ 6, 8, 7 }));
		m.groups[0].elements.push_back(Element({ 1, 8, 2 }));
		m.groups[0].elements.push_back(Element({ 1, 7, 8 }));
		m.groups[0].elements.push_back(Element({ 0, 6, 7 }));
		m.groups[0].elements.push_back(Element({ 0, 7, 1 }));
		m.groups[0].elements.push_back(Element({ 3, 8, 9 }));
		m.groups[0].elements.push_back(Element({ 3, 2, 8 }));

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
};

TEST_F(CollapserTest, collapser)
{
	Mesh m;
	m.grid = buildGridSize2();
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
	EXPECT_TRUE(r.groups[0].elements[0].isTriangle());
	EXPECT_TRUE(r.groups[0].elements[1].isTriangle());
}


TEST_F(CollapserTest, collapser_2)
{
	Mesh m = buildTinyTriMesh();

	Mesh r = Collapser(m, 2).getMesh();
	EXPECT_EQ(4, r.coordinates.size());
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(2, r.groups[0].elements.size());
	EXPECT_TRUE(r.groups[0].elements[0].isTriangle());
	EXPECT_TRUE(r.groups[0].elements[1].isTriangle());
}

TEST_F(CollapserTest, collapser_3)
{
	Mesh m = buildTinyTriMesh();

	Mesh r = Collapser(m, 4).getMesh();

	EXPECT_EQ(m.groups, r.groups);
	for (std::size_t i = 0; i < m.coordinates.size(); i++) {
		for (std::size_t d = 0; d < 3; d++) {
			EXPECT_NEAR(m.coordinates[i](d), r.coordinates[i](d), 1e-8) << "Current Coordinate: #" << i << std::endl;
		}
	}
}

TEST_F(CollapserTest, preserves_closedness)
{
	Mesh m = buildTinyTriClosedMesh();
	EXPECT_TRUE(isAClosedTopology(m.groups[0].elements));

	Mesh r = Collapser(m, 2).getMesh();
	
	EXPECT_EQ(8, r.coordinates.size());
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(12, r.groups[0].elements.size());

	EXPECT_TRUE(isAClosedTopology(r.groups[0].elements));
	
}

TEST_F(CollapserTest, closedness_for_sphere)
{
    auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
	for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }
	
	Mesh collapsed = m;
	GridTools gT{ m.grid };
    collapsed.coordinates = gT.absoluteToRelative(collapsed.coordinates);
    collapsed = Collapser{ collapsed, 4 }.getMesh();
    collapsed.coordinates = gT.relativeToAbsolute(collapsed.coordinates);

	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(collapsed.groups[0].elements));
	
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

	m = Slicer{ m }.getMesh();
	ASSERT_NO_THROW(Collapser(m, 2));
}


TEST_F(CollapserTest, round_lines_to_tolerance)
{
	int decimalPlaces = 2;
	auto tolerance = std::pow(10.0, decimalPlaces);

	Mesh mesh;
	mesh.grid = buildGridSize2();

	mesh.coordinates = {
		Coordinate({ -5.567, -18.5425, 1.0}),
		Coordinate({ 1.5972, -3.0, -2.0})
	};
	mesh.groups = { Group() };
	mesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line)
	};

	auto intermediateMesh = Slicer{ mesh }.getMesh();

	Mesh expectedMesh;
	expectedMesh.grid = buildGridSize2();

	for (auto& intermediateCoordinate : intermediateMesh.coordinates) {
		expectedMesh.coordinates.push_back(intermediateCoordinate.round(tolerance));
	}

	expectedMesh.groups.reserve(mesh.groups.size());

	for (auto& intermediateGroup : intermediateMesh.groups) {
		expectedMesh.groups.push_back(Group());

		auto& expectedGroup = expectedMesh.groups.back();

		expectedGroup.elements.reserve(intermediateGroup.elements.size());
		for (auto& intermediateElement : intermediateGroup.elements) {
			expectedGroup.elements.push_back(Element(intermediateElement.vertices, intermediateElement.type));
		}
	}

	auto resultMesh = Collapser(intermediateMesh, decimalPlaces).getMesh();

	ASSERT_EQ(resultMesh.coordinates.size(), expectedMesh.coordinates.size());

	for (std::size_t c = 0; c < resultMesh.coordinates.size(); ++c) {
		auto& expectedCoordinate = expectedMesh.coordinates[c];
		auto& resultCoordinate = resultMesh.coordinates[c];

		for (Axis axis = X; axis <= Z; ++axis) {
			EXPECT_EQ(resultCoordinate[axis], expectedCoordinate[axis])
				<< "Current coordinate: #" << c << std::endl
				<< "Current Axis: #" << axis << std::endl;
		}
	}

	ASSERT_EQ(resultMesh.groups.size(), expectedMesh.groups.size());
	ASSERT_EQ(resultMesh.groups.size(), mesh.groups.size());

	for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
		auto& expectedGroup = expectedMesh.groups[g];
		auto& resultGroup = resultMesh.groups[g];

		ASSERT_EQ(resultGroup.elements.size(), expectedGroup.elements.size()) << "Current Group: #" << g << std::endl;

		for (std::size_t e = 0; e < resultGroup.elements.size(); ++e) {
			auto & expectedElement = expectedGroup.elements[e];
			auto & resultElement = resultGroup.elements[e];

			ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;

			for (std::size_t v = 0; v < resultElement.vertices.size(); ++v) {
				EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
					<< "Current Group: #" << g << std::endl
					<< "Current Element: #" << e << std::endl
					<< "Current Vertex: #" << v << std::endl;
			}
		}
	}
}

TEST_F(CollapserTest, collapse_individual_lines_below_tolerance)
{
	int decimalPlaces = 2;
	auto tolerance = std::pow(10.0, decimalPlaces);

	Mesh mesh;
	mesh.grid = buildGridSize2();

	mesh.coordinates = {
		Coordinate({ 0.327, 1.355, 4.402}),
		Coordinate({ 0.33, 1.364, 4.404})
	};
	mesh.groups = { Group() };
	mesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line)
	};

	Mesh expectedMesh;
	expectedMesh.grid = buildGridSize2();

	
	expectedMesh.coordinates = {
		Coordinate({0.33, 1.36, 4.4})
	};
	

	expectedMesh.groups.resize(1);

	expectedMesh.groups[0].elements = { Element({0}, Element::Type::Node) };


	auto resultMesh = Collapser(mesh, decimalPlaces).getMesh();

	ASSERT_EQ(resultMesh.coordinates.size(), expectedMesh.coordinates.size());
	// ASSERT_EQ(resultMesh.coordinates.size(), 0);
	
	for (std::size_t c = 0; c < resultMesh.coordinates.size(); ++c) {
		auto& expectedCoordinate = expectedMesh.coordinates[c];
		auto& resultCoordinate = resultMesh.coordinates[c];

		for (Axis axis = X; axis <= Z; ++axis) {
			EXPECT_EQ(resultCoordinate[axis], expectedCoordinate[axis])
				<< "Current coordinate: #" << c << std::endl
				<< "Current Axis: #" << axis << std::endl;
		}
	}
	

	ASSERT_EQ(resultMesh.groups.size(), expectedMesh.groups.size());
	ASSERT_EQ(resultMesh.groups.size(), mesh.groups.size());


	for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
		auto& expectedGroup = expectedMesh.groups[g];
		auto& resultGroup = resultMesh.groups[g];

		ASSERT_EQ(resultGroup.elements.size(), expectedGroup.elements.size()) << "Current Group: #" << g << std::endl;
		
		for (std::size_t e = 0; e < resultGroup.elements.size(); ++e) {
			auto& expectedElement = expectedGroup.elements[e];
			auto& resultElement = resultGroup.elements[e];

			ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;

			for (std::size_t v = 0; v < resultElement.vertices.size(); ++v) {
				EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
					<< "Current Group: #" << g << std::endl
					<< "Current Element: #" << e << std::endl
					<< "Current Vertex: #" << v << std::endl;
			}
		}
	}
}


TEST_F(CollapserTest, equal_lines_get_erased_with_surface_policy)
{
	int decimalPlaces = 2;
	auto tolerance = std::pow(10.0, decimalPlaces);

	Mesh mesh;
	mesh.grid = buildGridSize2();

	mesh.coordinates = {
		Coordinate({ 0.000, 0.000, 0.000}),
		Coordinate({ 0.325, 0.325, 0.325}),
		Coordinate({ 1.557, 1.556, 1.584}),
		Coordinate({ 1.995, 3.226, 0.112}),
		Coordinate({ 1.562, 1.562, 1.579}),
		Coordinate({ 2.004, 3.230, 0.109}),
	};
	mesh.groups.resize(2);
	mesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({1, 4}, Element::Type::Line),
		Element({4, 5}, Element::Type::Line),
		Element({5, 4}, Element::Type::Line),
		Element({4, 1}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};
	mesh.groups[1].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({3, 4}, Element::Type::Line),
		Element({4, 5}, Element::Type::Line),
		Element({5, 4}, Element::Type::Line),
		Element({4, 3}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};

	Mesh expectedMesh;
	expectedMesh.grid = buildGridSize2();
	expectedMesh.coordinates = {
		Coordinate({ 0.00, 0.00, 0.00}),
		Coordinate({ 0.33, 0.33, 0.33}),
		Coordinate({ 1.56, 1.56, 1.58}),
		Coordinate({ 2.00, 3.23, 0.11}),
	};
	expectedMesh.groups.resize(2);
	expectedMesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
	};
	expectedMesh.groups[1].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
	};


	auto resultMesh = Collapser(mesh, decimalPlaces).getMesh();

	ASSERT_EQ(resultMesh.coordinates.size(), expectedMesh.coordinates.size());

	for (std::size_t c = 0; c < resultMesh.coordinates.size(); ++c) {
		auto& expectedCoordinate = expectedMesh.coordinates[c];
		auto& resultCoordinate = resultMesh.coordinates[c];

		for (Axis axis = X; axis <= Z; ++axis) {
			EXPECT_EQ(resultCoordinate[axis], expectedCoordinate[axis])
				<< "Current coordinate: #" << c << std::endl
				<< "Current Axis: #" << axis << std::endl;
		}
	}

	ASSERT_EQ(resultMesh.groups.size(), expectedMesh.groups.size());
	ASSERT_EQ(resultMesh.groups.size(), mesh.groups.size());

	for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
		auto& expectedGroup = expectedMesh.groups[g];
		auto& resultGroup = resultMesh.groups[g];

		ASSERT_EQ(resultGroup.elements.size(), expectedGroup.elements.size()) << "Current Group: #" << g << std::endl;

		for (std::size_t e = 0; e < resultGroup.elements.size(); ++e) {
			auto& expectedElement = expectedGroup.elements[e];
			auto& resultElement = resultGroup.elements[e];

			ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;

			for (std::size_t v = 0; v < resultElement.vertices.size(); ++v) {
				EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
					<< "Current Group: #" << g << std::endl
					<< "Current Element: #" << e << std::endl
					<< "Current Vertex: #" << v << std::endl;
			}
		}
	}
}


TEST_F(CollapserTest, equal_lines_get_erased_but_not_inverted_with_lines_policy)
{
	int decimalPlaces = 2;
	auto tolerance = std::pow(10.0, decimalPlaces);

	Mesh mesh;
	mesh.grid = buildGridSize2();

	mesh.coordinates = {
		Coordinate({ 0.000, 0.000, 0.000}),
		Coordinate({ 0.325, 0.325, 0.325}),
		Coordinate({ 1.557, 1.556, 1.584}),
		Coordinate({ 1.995, 3.226, 0.112}),
		Coordinate({ 1.562, 1.562, 1.579}),
		Coordinate({ 2.004, 3.230, 0.109}),
	};
	mesh.groups.resize(2);
	mesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({1, 4}, Element::Type::Line),
		Element({4, 5}, Element::Type::Line),
		Element({5, 4}, Element::Type::Line),
		Element({4, 1}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};
	mesh.groups[1].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({3, 4}, Element::Type::Line),
		Element({4, 5}, Element::Type::Line),
		Element({5, 4}, Element::Type::Line),
		Element({4, 3}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};

	Mesh expectedMesh;
	expectedMesh.grid = buildGridSize2();
	expectedMesh.coordinates = {
		Coordinate({ 0.00, 0.00, 0.00}),
		Coordinate({ 0.33, 0.33, 0.33}),
		Coordinate({ 1.56, 1.56, 1.58}),
		Coordinate({ 2.00, 3.23, 0.11}),
	};
	expectedMesh.groups.resize(2);
	expectedMesh.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({2, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};
	expectedMesh.groups[1].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
	};


	auto resultMesh = Collapser(mesh, decimalPlaces, {Element::Type::Line, Element::Type::Surface}).getMesh();

	ASSERT_EQ(resultMesh.coordinates.size(), expectedMesh.coordinates.size());

	for (std::size_t c = 0; c < resultMesh.coordinates.size(); ++c) {
		auto& expectedCoordinate = expectedMesh.coordinates[c];
		auto& resultCoordinate = resultMesh.coordinates[c];

		for (Axis axis = X; axis <= Z; ++axis) {
			EXPECT_EQ(resultCoordinate[axis], expectedCoordinate[axis])
				<< "Current coordinate: #" << c << std::endl
				<< "Current Axis: #" << axis << std::endl;
		}
	}

	ASSERT_EQ(resultMesh.groups.size(), expectedMesh.groups.size());
	ASSERT_EQ(resultMesh.groups.size(), mesh.groups.size());

	for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
		auto& expectedGroup = expectedMesh.groups[g];
		auto& resultGroup = resultMesh.groups[g];

		ASSERT_EQ(resultGroup.elements.size(), expectedGroup.elements.size()) << "Current Group: #" << g << std::endl;

		for (std::size_t e = 0; e < resultGroup.elements.size(); ++e) {
			auto& expectedElement = expectedGroup.elements[e];
			auto& resultElement = resultGroup.elements[e];

			ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;

			for (std::size_t v = 0; v < resultElement.vertices.size(); ++v) {
				EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
					<< "Current Group: #" << g << std::endl
					<< "Current Element: #" << e << std::endl
					<< "Current Vertex: #" << v << std::endl;
			}
		}
	}
}

}