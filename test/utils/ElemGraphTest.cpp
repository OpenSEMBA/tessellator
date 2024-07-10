#include "gtest/gtest.h"

#include "utils/ElemGraph.h"


namespace meshlib {
namespace utils {

class ElemGraphTest : public ::testing::Test {
public:
	static auto getElementsView(const Elements& elems) {
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}
		return elemPtrs;
	}
};

TEST_F(ElemGraphTest, getVertices)
{
	//   3 - 2
	//   | / |
	//   0 - 1
	Coordinates coords =
	{
		Coordinate({ 0.00, 0.00, 0.00 }),
		Coordinate({ 1.00, 0.00, 0.00 }),
		Coordinate({ 0.00, 1.00, 0.00 }),
		Coordinate({ 1.00, 1.00, 0.00 })
	};

	Elements elems =
	{
		Element({ 0, 1, 2 }, Element::Type::Surface),
		Element({ 0, 2, 3 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);
	EXPECT_EQ(2, g.getVertices().size());
}

TEST_F(ElemGraphTest, ctor_with_lines) 
{
	Coordinates coords = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({2.0, 0.0, 0.0}),
	};

	Elements lines =
	{
		Element({ 0, 1 }, Element::Type::Line),
		Element({ 1, 2 }, Element::Type::Line)
	};

	ASSERT_NO_THROW(ElemGraph g(lines, coords));
}

TEST_F(ElemGraphTest, ctor_fails_when_mixing_lines_and_tris)
{
	Coordinates coords = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({2.0, 0.0, 0.0}),
		Coordinate({1.0, 1.0, 0.0}),
	};

	Elements elems = {
		Element({ 0, 1 },    Element::Type::Line),
		Element({ 1, 2, 3 }, Element::Type::Surface)
	};

	ASSERT_ANY_THROW(ElemGraph g(elems, coords));
}

TEST_F(ElemGraphTest, getAsElements) 
{
	//             3
	//           /
	// 0 - 1 - 2 

	Coordinates cs = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({2.0, 0.0, 0.0}),
		Coordinate({3.0, 2.0, 0.0})
	};

	Elements es = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line)
	};

	ElemGraph eG(es, cs);

	EXPECT_EQ(es, eG.getAsElements(es));
}
TEST_F(ElemGraphTest, splitByWeight_for_lines) 
{
	//             3
	//           /
	// 0 - 1 - 2 

	Coordinates cs = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({2.0, 0.0, 0.0}),
		Coordinate({3.0, 2.0, 0.0})
	};
	
	Elements es = {
		Element({0, 1}, Element::Type::Line),
		Element({1, 2}, Element::Type::Line),
		Element({2, 3}, Element::Type::Line)
	};

	ElemGraph eG(es, cs);
	
	EXPECT_EQ(1, eG.splitByWeight(80).size());
	EXPECT_EQ(2, eG.splitByWeight(45).size());
	EXPECT_EQ(2, eG.splitByWeight(20).size());
}

TEST_F(ElemGraphTest, adjacentVertices)
{
	//   2 - 3
	//   | / |
	//   0 - 1
	Coordinates coords =
	{
		Coordinate({ 0.00, 0.00, 0.00 }),
		Coordinate({ 1.00, 0.00, 0.00 }),
		Coordinate({ 0.00, 1.00, 0.00 }),
		Coordinate({ 1.00, 1.00, 0.00 })
	};

	Elements elems =
	{
		Element({ 0, 1, 3 }, Element::Type::Surface),
		Element({ 0, 3, 2 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);

	EXPECT_EQ(1, g.getAdjacentVertices(0).size());
	EXPECT_EQ(1, g.getAdjacentVertices(0).count(1));
	EXPECT_EQ(1, g.getAdjacentVertices(1).size());
	EXPECT_EQ(1, g.getAdjacentVertices(1).count(0));

}

TEST_F(ElemGraphTest, graph_with_one_element)
{
	//   2 
	//   | \ 
	//   0 - 1 
	std::vector<Coordinate> coords =
	{
		Coordinate({ 0.00, 0.00, 0.00 }),
		Coordinate({ 1.00, 0.00, 0.00 }),
		Coordinate({ 0.00, 1.00, 0.00 })
	};

	std::vector<Element> elems =
	{
		Element({ 0, 1, 2 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);

	EXPECT_EQ(1, g.getVertices().size());
	EXPECT_EQ(1, g.split().size());
	EXPECT_EQ(1, g.splitByWeight(80).size());
}

TEST_F(ElemGraphTest, empty_graph_split)
{
	ElemGraph g;
	
	std::vector<ElemGraph> sG;
	ASSERT_NO_THROW(auto sG = g.split());

	EXPECT_EQ(0, sG.size());
}

TEST_F(ElemGraphTest, splitByWeight_90_deg)
{
	//   2 - 3
	//   | / |
	//   0 - 1
	//   90 degrees angle
	Coordinates coords =
	{
		Coordinate({ 0.00, 0.00, 0.00 }),
		Coordinate({ 0.50, 0.50, 1.00 }),
		Coordinate({ 0.00, 1.00, 0.00 }),
		Coordinate({ 1.00, 1.00, 0.00 })
	};

	Elements elems =
	{
		Element({ 0, 1, 3 }, Element::Type::Surface),
		Element({ 0, 3, 2 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);
	EXPECT_EQ(2, g.getVertices().size());
	EXPECT_EQ(2, g.splitByWeight(80).size());
}

TEST_F(ElemGraphTest, splitByWeight_minus90deg)
{
	//   2 - 3
	//   | / |
	//   0 - 1
	//   90 degrees angle
	Coordinates coords =
	{
		Coordinate({ 0.00, 0.00, 0.00 }),
		Coordinate({ 0.50, 0.50, -1.00 }),
		Coordinate({ 0.00, 1.00, 0.00 }),
		Coordinate({ 1.00, 1.00, 0.00 })
	};

	Elements elems =
	{
		Element({ 0, 1, 3 }, Element::Type::Surface),
		Element({ 0, 3, 2 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);
	EXPECT_EQ(2, g.getVertices().size());
	EXPECT_EQ(2, g.splitByWeight(80).size());
}


TEST_F(ElemGraphTest, splitByWeight_2)
{
	//   3 - 1
	//   | / |
	//   0 - 2
	Coordinates coords =
	{
		Coordinate({ 22.032786885245901, 13.812018490169491, 2.0000000000000000 }),
		Coordinate({ 22.032786885245901, 14.000000000000000, 1.9802397151103044	}),
		Coordinate({ 22.000000000000000, 13.838932591707717, 1.9973032591892008	}),
		Coordinate({ 22.000000000000000, 14.000000000000000, 1.9676392093021247 })
	};
	Elements elems =
	{
		Element({ 2, 0, 1 }, Element::Type::Surface),
		Element({ 0, 1, 3 }, Element::Type::Surface)
	};

	ElemGraph g(getElementsView(elems), coords);

	EXPECT_EQ(2, g.getVertices().size());
	EXPECT_EQ(2, g.splitByWeight(30.0).size());
}

TEST_F(ElemGraphTest, noSplitnoDisjoint)
{
	{
		ElemGraph g;
		g.addEdge(0, 2, 0);
		g.addEdge(1, 2, 0);
		EXPECT_EQ(1, g.split().size());
	}
	{
		ElemGraph g;
		g.addEdge(0, 2, 0);
		g.addEdge(2, 1, 0);
		EXPECT_EQ(1, g.split().size());
	}
}



}
}