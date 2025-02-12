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
