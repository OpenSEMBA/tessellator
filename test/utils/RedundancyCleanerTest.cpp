#include <algorithm>
#include "gtest/gtest.h"
#include <cmath>

#include "RedundancyCleaner.h"
#include "MeshFixtures.h"

namespace meshlib::utils {

using namespace meshFixtures;

class RedundancyCleanerTest : public ::testing::Test {
public:
	static std::size_t countDifferent(const Coordinates& cs)
	{
		return std::set<Coordinate>(cs.begin(), cs.end()).size();
	}
};

TEST_F(RedundancyCleanerTest, removeRepeatedElements)
{
	auto m{ buildCubeSurfaceMesh(1.0) };
	
	auto r{ m };
	r.groups[0].elements.push_back(m.groups[0].elements.back());

	RedundancyCleaner::removeRepeatedElements(r);

	EXPECT_EQ(m, r);
}

TEST_F(RedundancyCleanerTest, removeRepeatedElements_with_indices_rotated)
{
	auto m{ buildCubeSurfaceMesh(1.0) };

	auto r{ m };
	r.groups[0].elements.push_back(m.groups[0].elements.back());
	auto& e = r.groups[0].elements.back();
	std::rotate(e.vertices.begin(), e.vertices.begin() + 1, e.vertices.end());

	RedundancyCleaner::removeRepeatedElements(r);

	EXPECT_EQ(m, r);
}

TEST_F(RedundancyCleanerTest, testRemoveRepeatedLinesFromSameGroup)
{
	Mesh m;
	m.grid = buildUnitLengthGrid(0.2);
	m.coordinates = {
		Coordinate({0.25, 0.25, 0.25}),
		Coordinate({0.75, 0.75, 0.75}),
	};
	m.groups.resize(2);
	m.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({0, 1}, Element::Type::Line),
	};
	m.groups[1].elements = {
		Element({0, 1}, Element::Type::Line),
	};

	RedundancyCleaner::removeRepeatedElements(m);

	EXPECT_EQ(m.coordinates.size(), 2);
	EXPECT_EQ(m.groups.size(), 2);
	EXPECT_EQ(m.groups[0].elements.size(), 1);
	EXPECT_EQ(m.groups[1].elements.size(), 1);
}


TEST_F(RedundancyCleanerTest, testRemoveOverlappedElementsContainedWithinLines)
{
	Mesh m;
	m.grid = buildUnitLengthGrid(0.2);
	m.coordinates = {
		Coordinate({0.0, 0.0, 0.0}), // 0
		Coordinate({0.0, 1.0, 0.0}), // 1
		Coordinate({1.0, 1.0, 0.0}), // 2
		Coordinate({1.0, 0.0, 0.0}), // 3
		Coordinate({0.0, 0.0, 1.0}), // 4
		Coordinate({0.0, 1.0, 1.0}), // 5
		Coordinate({1.0, 1.0, 1.0}), // 6
		Coordinate({1.0, 0.0, 1.0}), // 7
	};
	m.groups.resize(3);
	m.groups[0].elements = {
		Element({0}, Element::Type::Node),
		Element({1}, Element::Type::Node),
		Element({4}, Element::Type::Node),
		Element({5}, Element::Type::Node),
		Element({5}, Element::Type::Node),
		Element({0, 1}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line),
		Element({3, 2}, Element::Type::Line),
		Element({7, 6}, Element::Type::Line),
		
	};
	m.groups[1].elements = {
		Element({0}, Element::Type::Node),
		Element({1}, Element::Type::Node),
		Element({2, 3}, Element::Type::Line),
	};
	m.groups[2].elements = {
		Element({0}, Element::Type::Node),
		Element({1}, Element::Type::Node),
		Element({0}, Element::Type::Node),
	};

	Coordinates expectedCoordinates{m.coordinates};

	std::vector<Elements> expectedElementsList = {
		{
			Element({4}, Element::Type::Node),
			Element({5}, Element::Type::Node),
			Element({0, 1}, Element::Type::Line),
			Element({2, 3}, Element::Type::Line),
			Element({3, 2}, Element::Type::Line),
			Element({7, 6}, Element::Type::Line),
		},
		{
			Element({0}, Element::Type::Node),
			Element({1}, Element::Type::Node),
			Element({2, 3}, Element::Type::Line),
		},
		{
			Element({0}, Element::Type::Node),
			Element({1}, Element::Type::Node),
		}
	};

	RedundancyCleaner::removeOverlappedElementsForLineMeshing(m);

	EXPECT_EQ(m.coordinates.size(), expectedCoordinates.size());
	ASSERT_EQ(m.groups.size(), expectedElementsList.size());

	for(std::size_t index = 0; index < m.coordinates.size(); ++index){
		const auto & resultCoordinate = m.coordinates[index];
		const auto & expectedCoordinate = expectedCoordinates[index];

		for(Axis axis = X; axis <= Z; ++axis){
			EXPECT_FLOAT_EQ(resultCoordinate[axis], expectedCoordinate[axis]) << "Current Coordinate: #" << index << std::endl;
		}
	}

	for(std::size_t g = 0; g < m.groups.size(); ++g){
		const auto & resultElements = m.groups[g].elements;
		const auto & expectedElements = expectedElementsList[g];

		ASSERT_EQ(resultElements.size(), expectedElements.size()) << "Current Group: #" << g << std::endl;

		for(std::size_t e = 0; e < resultElements.size(); ++e){
			const auto & resultElement = resultElements[e];
			const auto & expectedElement = expectedElements[e];
			
			EXPECT_EQ(resultElement.type, expectedElement.type);
			ASSERT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;

			for(std::size_t v = 0; v < resultElement.vertices.size(); ++v){
				EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl
				<< "Current Vertex: #" << v << std::endl;
			}
		}
	}
}

TEST_F(RedundancyCleanerTest, testDoNotRemoveOppositeLines)
{
	Mesh m;
	m.grid = buildUnitLengthGrid(0.2);
	m.coordinates = {
		Coordinate({0.25, 0.25, 0.25}),
		Coordinate({0.75, 0.75, 0.75}),
	};
	m.groups.resize(1);
	m.groups[0].elements = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 0}, Element::Type::Line),
	};

	auto resultMesh{ m };

	RedundancyCleaner::removeRepeatedElements(resultMesh);

	EXPECT_EQ(resultMesh, m);
}

TEST_F(RedundancyCleanerTest, removeElementsWithCondition)
{
	Mesh m;
	m.coordinates = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({0.0, 1.0, 0.0}),
		Coordinate({2.0, 0.0, 0.0}),
	};

	m.groups = { Group() };
	m.groups[0].elements = {
		Element({0, 1, 2}, Element::Type::Surface),
		Element({2, 3}, Element::Type::Line),
		Element({}, Element::Type::None)
	};

	
	{
		Mesh r = m;
		RedundancyCleaner::removeElementsWithCondition(r, [](auto e) {return e.isNone(); });

		EXPECT_EQ(3, m.groups[0].elements.size());
		EXPECT_EQ(2, r.groups[0].elements.size());
		EXPECT_EQ(m.coordinates, r.coordinates);
	}

	{
		Mesh r = m;
		RedundancyCleaner::removeElementsWithCondition(
			r, [](auto e) {return e.isLine() || e.isNone(); });

		EXPECT_EQ(3, m.groups[0].elements.size());
		EXPECT_EQ(1, r.groups[0].elements.size());
		EXPECT_EQ(m.coordinates, r.coordinates);

	}

}

}
