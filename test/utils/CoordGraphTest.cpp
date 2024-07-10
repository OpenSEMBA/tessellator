#include "gtest/gtest.h"

#include "utils/CoordGraph.h"


namespace meshlib {
namespace utils {

class CoordGraphTest : public ::testing::Test {
public:
	static auto getElementsView(const Elements& elems) {
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}
		return elemPtrs;
	}
};

TEST_F(CoordGraphTest, getVertices)
{
	CoordGraph g;
	g.addVertex(1);
	g.addVertex(5);

	EXPECT_EQ(2, g.getVertices().size());
}

TEST_F(CoordGraphTest, path_constructor)
{
	//12 - 0 - 1    7 - 8   11 - 10
	//     | \ |    | /
	//     3 - 2    9
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 0);
	g.addEdge(0, 2);
	g.addEdge(2, 0);
	g.addEdge(7, 8);
	g.addEdge(8, 9);
	g.addEdge(9, 7);
	g.addEdge(10, 11);
	g.addEdge(0, 12);
	
	auto cycles = g.findCycles();
	auto aPaths = g.findAcyclicPaths();
	CoordGraph::Paths paths; 
	paths.insert(paths.begin(), cycles.begin(), cycles.end());
	paths.insert(paths.begin(), aPaths.begin(), aPaths.end());
	CoordGraph gPaths(paths);
	ASSERT_EQ(gPaths.findCycles().size(), cycles.size());
	ASSERT_EQ(gPaths.findAcyclicPaths().size(), aPaths.size());
}
TEST_F(CoordGraphTest, adjacentVertices)
{
	CoordGraph g;
	g.addEdge(3, 4);
	g.addEdge(4, 5);

	EXPECT_EQ(1, g.getAdjacentVertices(3).size());
	EXPECT_EQ(1, g.getAdjacentVertices(3).count(4));

	EXPECT_EQ(2, g.getAdjacentVertices(4).size());
	EXPECT_EQ(1, g.getAdjacentVertices(4).count(3));
	EXPECT_EQ(1, g.getAdjacentVertices(4).count(5));
	
	EXPECT_EQ(1, g.getAdjacentVertices(5).size());
	EXPECT_EQ(1, g.getAdjacentVertices(5).count(4));
}

TEST_F(CoordGraphTest, getBoundaryGraph_1)
{
	//   7 - 3
	//   | / |
	//   6 - 2
	//   | \ |
	//   5 - 1 - 0
	//     \ | /
	//       4 
	std::vector<Element> elems;
	{
		Element triangle;
		triangle.vertices = { 1, 0, 4 };
		elems.push_back(triangle);
		triangle.vertices = { 4, 5, 1 };
		elems.push_back(triangle);
		triangle.vertices = { 1, 6, 2 };
		elems.push_back(triangle);
		triangle.vertices = { 1, 5, 6 };
		elems.push_back(triangle);
		triangle.vertices = { 2, 6, 3 };
		elems.push_back(triangle);
		triangle.vertices = { 6, 7, 3 };
		elems.push_back(triangle);
	}
	
	CoordGraph boundary = CoordGraph(getElementsView(elems)).getBoundaryGraph();
	auto cycles = boundary.findCycles();
	EXPECT_EQ(1, cycles.size());
	auto paths = boundary.findAcyclicPaths();
	EXPECT_EQ(0, paths.size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_2)
{
	// 0 - 1 - 2 - 3
	CoordGraph boundary;
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 3);
		boundary = g.getBoundaryGraph();
	}

	auto cycles = boundary.findCycles();
	EXPECT_EQ(0, cycles.size());

	auto paths = boundary.findAcyclicPaths();
	EXPECT_EQ(1, paths.size());
	EXPECT_EQ(4, paths.front().size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_3)
{
	// 0 - 1 - 2 - 3
	// |           |
	// 4 - 5 - 6 - 7
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 7);
	g.addEdge(7, 6);
	g.addEdge(6, 5);
	g.addEdge(5, 4);
	g.addEdge(4, 0);

	CoordGraph boundary = g.getBoundaryGraph();
	EXPECT_EQ(8, boundary.verticesSize());

	auto cycles = boundary.findCycles();
	EXPECT_EQ(1, cycles.size());

	auto paths = boundary.findAcyclicPaths();
	EXPECT_EQ(0, paths.size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_4)
{
	// 0 - 1    7 - 8
	// | \ |    | /
	// 3 - 2    9
	CoordGraph g;
	g.addEdge(0,1);
	g.addEdge(1,2);
	g.addEdge(2,3);
	g.addEdge(3,0);
	g.addEdge(0,2);
	g.addEdge(2,0);
	g.addEdge(7,8);
	g.addEdge(8,9);
	g.addEdge(9,7);

	CoordGraph boundary = g.getBoundaryGraph();
	EXPECT_EQ(7, boundary.verticesSize());

	auto cycles = boundary.findCycles();
	EXPECT_EQ(2, cycles.size());

	auto paths = boundary.findAcyclicPaths();
	EXPECT_EQ(0, paths.size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_5)
{
	// 0 ----- 3
	// | \   / |
	// |   4   |
	// | /   \ |
    // 1 ----- 2
	std::vector<Element> elems;
	{
		Element t;
		t.vertices = { 0, 1, 4 };    elems.push_back(t);
		t.vertices = { 1, 2, 4 };    elems.push_back(t);
		t.vertices = { 2, 3, 4 };    elems.push_back(t);
		t.vertices = { 3, 0, 4 };    elems.push_back(t);
	}
	
	auto cycles = CoordGraph(getElementsView(elems)).findCycles();
	
	CoordGraph boundary = CoordGraph(getElementsView(elems)).getBoundaryGraph();
	EXPECT_EQ(4, boundary.verticesSize());

	EXPECT_EQ(IdSet({0, 1, 2, 3}), boundary.getVertices());
    EXPECT_EQ(1, boundary.findCycles().size());
	EXPECT_EQ(0, boundary.findAcyclicPaths().size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_6_alt)
{
	// 0 ----- 3
	// | \     |
	// |   \   |
	// |     \ |
	// 1 ----- 2
	std::vector<Element> elems;
	{
		Element t;
		t.vertices = { 0, 1, 2 };    elems.push_back(t);
		t.vertices = { 2, 3, 0 };    elems.push_back(t);
	}

	CoordGraph boundary = CoordGraph(getElementsView(elems)).getBoundaryGraph();
	EXPECT_EQ(IdSet({ 0, 1, 2, 3 }), boundary.getVertices());
	EXPECT_EQ(1, boundary.findCycles().size());
	EXPECT_EQ(0, boundary.findAcyclicPaths().size());
}

TEST_F(CoordGraphTest, getBoundaryGraph_6) 
{

	// 0 ----- 1
	// | \     |
	// |   \   |
	// |     \ |
	// 3 ----- 2 -- 5
	// |
	// 4


	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 0);
	g.addEdge(3, 4);
	g.addEdge(2, 5);
	CoordGraph boundary = g.getBoundaryGraph();
	EXPECT_EQ(1, boundary.findCycles().size());
	EXPECT_EQ(2, boundary.findAcyclicPaths().size());

}

TEST_F(CoordGraphTest, getBoundaryGraph_7) 
{

	// 6 -  9 
	// | \  | \
	// 8 - 13 - 1 
	// | \  | /
	// 4 - 11

	Elements es = {
		Element({ 6,  8, 13}, Element::Type::Surface),
		Element({ 6, 13,  9}, Element::Type::Surface),
		Element({ 8,  4, 11}, Element::Type::Surface),
		Element({ 8, 11, 13}, Element::Type::Surface),
		Element({ 1, 13, 11}, Element::Type::Surface),
		Element({ 1,  9, 13}, Element::Type::Surface)
	};
	
	IdSet contour = CoordGraph(getElementsView(es)).getBoundaryGraph().getVertices();
	EXPECT_EQ(IdSet({ 1, 4, 6, 8, 9, 11 }), contour);
}

TEST_F(CoordGraphTest, getBoundaryGraph_8)
{

	// 24    4 
	// |  \  | \
	// 19 - 12 - 16 

	Elements es = {
		Element({ 24, 12, 19}, Element::Type::Surface),
		Element({ 12,  4, 16}, Element::Type::Surface)
	};

	IdSet contour = CoordGraph(getElementsView(es)).getBoundaryGraph().getVertices();
	EXPECT_EQ(IdSet({ 4, 12, 16, 19, 24 }), contour);
}

TEST_F(CoordGraphTest, getInternalGraph_1)
{

	// 8  -  10 
	// |  \  |
	// 13 -  9
	Elements es = {
		Element({ 8, 10,  9}, Element::Type::Surface),
		Element({ 8,  9, 13}, Element::Type::Surface)
	};

	auto g = CoordGraph(getElementsView(es)).getInternalGraph();
	EXPECT_EQ(IdSet({ 8, 9 }), g.getVertices());
}

TEST_F(CoordGraphTest, getInternalGraph_2)
{

	// 6 -  9 
	// | \  | \
	// 8 - 13 - 1 
	// | \  | /
	// 4 - 11

	Elements es = {
		Element({ 6,  8, 13}, Element::Type::Surface),
		Element({ 6, 13,  9}, Element::Type::Surface),
		Element({ 8,  4, 11}, Element::Type::Surface),
		Element({ 8, 11, 13}, Element::Type::Surface),
		Element({ 1, 13, 11}, Element::Type::Surface),
		Element({ 1,  9, 13}, Element::Type::Surface)
	};

	auto g = CoordGraph(getElementsView(es)).getInternalGraph();
	EXPECT_EQ(IdSet({ 1, 6, 8, 9, 11, 13 }), g.getVertices());
	EXPECT_EQ(12, g.edgesSize());
}


TEST_F(CoordGraphTest, ctors)
{
	// 0 ----- 3
	// | \     |
	// |   \   |
	// |     \ |
	// 1 ----- 2
	std::vector<Element> elems;
	{
		Element t;
		t.vertices = { 0, 1, 2 };    elems.push_back(t);
		t.vertices = { 2, 3, 0 };    elems.push_back(t);
	}

	CoordGraph cG(elems);
	CoordGraph cGViews(getElementsView(elems));
	EXPECT_EQ(cG.verticesSize(), cGViews.verticesSize());
	EXPECT_EQ(cG.edgesSize(), cGViews.edgesSize());
}
TEST_F(CoordGraphTest, adding_edge_with_non_existingvertices)
{
	CoordGraph g;
	g.addEdge(0, 1);
	EXPECT_EQ(2, g.getVertices().size());
	EXPECT_EQ(1, g.findAcyclicPaths().size());
}
TEST_F(CoordGraphTest, throw_when_adding_selfEdges) 
{
	CoordGraph g;
	g.addVertex(0);
	EXPECT_ANY_THROW(g.addEdge(0, 0));
}

TEST_F(CoordGraphTest, remove_edge)
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 0);

	EXPECT_EQ(1, g.findCycles().size());
	g.removeEdge(0, 1);
	EXPECT_EQ(0, g.findCycles().size());
	EXPECT_EQ(1, g.findAcyclicPaths().size());
}

TEST_F(CoordGraphTest, getEdgesAsLines)
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 0);

	Elements lines = g.getEdgesAsLines();
	
	ASSERT_EQ(3, lines.size());
	
	EXPECT_EQ(Element::Type::Line, lines[0].type);
	EXPECT_EQ(Element::Type::Line, lines[1].type);
	EXPECT_EQ(Element::Type::Line, lines[2].type);

	EXPECT_EQ(std::vector<CoordinateId>({0, 1}), lines[0].vertices);
	EXPECT_EQ(std::vector<CoordinateId>({1, 2}), lines[1].vertices);
	EXPECT_EQ(std::vector<CoordinateId>({2, 0}), lines[2].vertices);
}


TEST_F(CoordGraphTest, getEdgesAsLines_2)
{
	// 0 ----- 3
	// | \     |
	// |   \   |
	// |     \ |
	// 1 ----- 2
	std::vector<Element> elems;
	{
		Element t;
		t.vertices = { 0, 1, 2 };    elems.push_back(t);
		t.vertices = { 2, 3, 0 };    elems.push_back(t);
	}
	CoordGraph g(elems);

	Elements lines = g.getEdgesAsLines();

	EXPECT_EQ(6, lines.size());
}
TEST_F(CoordGraphTest, empty_graph_split)
{
	CoordGraph g;

	std::vector<CoordGraph> sG;
	ASSERT_NO_THROW(auto sG = g.split());

	EXPECT_EQ(0, sG.size());
}

TEST_F(CoordGraphTest, split_1) 
{
	CoordGraph g;
	g.addEdge(10, 11);
	g.addVertex(12);
	
	EXPECT_EQ(2, g.split().size());
}

TEST_F(CoordGraphTest, split_2)
{
	CoordGraph g;
	g.addEdge(10, 11);
	g.addEdge(12, 13);

	EXPECT_EQ(2, g.split().size());
}
TEST_F(CoordGraphTest, edge_split)
{
	CoordGraph g;
	g.addEdge(10, 11);

	EXPECT_EQ(1, g.split().size());
}

TEST_F(CoordGraphTest, splitDisjointGraphs)
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 0);
	g.addEdge(7, 8);
	g.addEdge(8, 9);
	g.addEdge(9, 7);
	EXPECT_EQ(2, g.split().size());
	EXPECT_EQ(1, g.split()[0].findCycles().size());
	EXPECT_EQ(1, g.split()[1].findCycles().size());


}

TEST_F(CoordGraphTest, intersect_edge) 
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);

	EXPECT_EQ(4, g.verticesSize());
	EXPECT_EQ(3, g.edgesSize());

	IdSet intersectId = { 1,2 };
	CoordGraph gIntersect = g.intersect(intersectId);
	EXPECT_EQ(2, gIntersect.verticesSize());
	EXPECT_EQ(1, gIntersect.edgesSize());
}

TEST_F(CoordGraphTest, intersect_edge_2)
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 4);
	g.addEdge(4, 5);
	EXPECT_EQ(6, g.verticesSize());
	EXPECT_EQ(5, g.edgesSize());

	IdUSet intersectId = { 2, 1, 8 };
	CoordGraph gIntersect = g.intersect(intersectId);
	EXPECT_EQ(2, gIntersect.verticesSize());
	EXPECT_EQ(1, gIntersect.edgesSize());
}

TEST_F(CoordGraphTest, intersect_face) {
	{
		std::vector<Element> elems;
		{
			Element triangle;
			triangle.vertices = { 1, 0, 4 };
			elems.push_back(triangle);
			triangle.vertices = { 4, 1, 5 };
			elems.push_back(triangle);
			triangle.vertices = { 1, 6, 2 };
			elems.push_back(triangle);
			triangle.vertices = { 1, 5, 6 };
			elems.push_back(triangle);
			triangle.vertices = { 2, 6, 3 };
			elems.push_back(triangle);
			triangle.vertices = { 6, 7, 3 };
			elems.push_back(triangle);
		}
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}

		IdSet coordsOnX0;
		coordsOnX0.insert(0);
		coordsOnX0.insert(1);
		coordsOnX0.insert(2);
		coordsOnX0.insert(3);

		CoordGraph g(elemPtrs);
		CoordGraph graphX0 = g.intersect(coordsOnX0);
		auto cycles = graphX0.findCycles();
		EXPECT_EQ(0, cycles.size());
		auto paths = graphX0.findAcyclicPaths();
		EXPECT_EQ(1, paths.size());
		EXPECT_EQ(4, paths.front().size());
	}

}

TEST_F(CoordGraphTest, getClosestVerticesInSet_1)
{
	//  1 - 3 
	//  | / |
	//  2 - 4
	CoordGraph g;
	g.addEdge(1, 2);
	g.addEdge(1, 3);
	g.addEdge(2, 3);
	g.addEdge(2, 4);
	g.addEdge(3, 4);

	EXPECT_EQ(IdSet({ 2, 3 }), g.getClosestVerticesInSet(1, { 2, 3, 4 }));
	EXPECT_EQ(IdSet({ 2, 3 }), g.getClosestVerticesInSet(4, { 1, 2, 3 }));
	
	EXPECT_EQ(IdSet({ 2 }), g.getClosestVerticesInSet(1, { 2 }));
}


TEST_F(CoordGraphTest, getClosestVerticesInSet_2)
{
	// 1 - 2 - 3 - 4
	CoordGraph g;
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 4);

	EXPECT_EQ(IdSet({ 2 }), g.getClosestVerticesInSet(1, { 2, 3, 4 }));
	EXPECT_EQ(IdSet({ 3 }), g.getClosestVerticesInSet(2, { 3, 4 }));
	
}
TEST_F(CoordGraphTest, graphIntersection) 
{

	CoordGraph g1;
	g1.addEdge(2, 3);
	g1.addEdge(3, 4);
	g1.addEdge(4, 2);
	g1.addEdge(4, 3);
	g1.addEdge(3, 5);
	g1.addEdge(5, 4);
	CoordGraph g2;
	//g2.addEdge(2, 3);
	//g2.addEdge(3, 0);
	//g2.addEdge(0, 2);
	g2.addEdge(4, 5);
	g2.addEdge(5, 1);
	g2.addEdge(1, 4);

	CoordGraph intersection = g1.intersect(g2);
	auto cycles = intersection.findCycles();
	EXPECT_EQ(0, cycles.size());
	auto paths = intersection.findAcyclicPaths();
	EXPECT_EQ(1, paths.size());
	EXPECT_EQ(2, paths.front().size());
	//EXPECT_EQ(2, paths.back().size());

}
TEST_F(CoordGraphTest, graphDifference_2)
{
	// 10 -- 11 -- 12 -- 15
	//        \   /
	//          14
	CoordGraph g;
	g.addEdge(10, 11);
	g.addEdge(11, 12);
	g.addEdge(12, 14);
	g.addEdge(14, 11);
	g.addEdge(12, 15);
	CoordGraph gCycle;
	gCycle.addEdge(11, 12);
	gCycle.addEdge(12, 14);
	gCycle.addEdge(14, 11);

	auto gDiff = g.difference(gCycle);

	ASSERT_EQ(0, gDiff.findCycles().size());
	ASSERT_EQ(2, gDiff.findAcyclicPaths().size());
	ASSERT_EQ(2, gDiff.split().size());
}


TEST_F(CoordGraphTest, orientedIntersection) {

	{
		std::vector<Element> elems;
		{
			Element triangle;
			triangle.vertices = { 1, 0, 2 };
			elems.push_back(triangle);
		}
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}

		CoordGraph g(elemPtrs);
		CoordGraph graphX0 = g.intersect(IdSet({ 0, 1 }));
		auto cycles = graphX0.findCycles();
		auto paths = graphX0.findAcyclicPaths();
		for (auto& path : paths) {
			path = graphX0.orderByOrientation(path);
		}

		CoordGraph::Path expected = { 1, 0 };
		ASSERT_EQ(1, paths.size());
		EXPECT_EQ(paths[0], expected);

	}

	{
		CoordGraph g;
		g.addEdge(1, 0);
		g.addEdge(0, 2);
		g.addEdge(2, 1);

		CoordGraph graphX0 = g.intersect(IdSet({ 0, 1 }));
		auto cycles = graphX0.findCycles();
		auto paths = graphX0.findAcyclicPaths();
		for (auto& path : paths) {
			path = graphX0.orderByOrientation(path);
		}

		CoordGraph::Path expected = { 1, 0 };
		ASSERT_EQ(1, paths.size());
		EXPECT_EQ(paths[0], expected);
	}
}
TEST_F(CoordGraphTest, repeatedVerticesAndEdges) {
	{
		CoordGraph g;
		g.addVertex(0);
		g.addVertex(1);
		g.addVertex(1);
		g.addVertex(1);
		g.addVertex(2);
		g.addEdge(0, 1);
		g.addEdge(1, 2);

		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(1, paths.size());
		EXPECT_EQ(3, paths.front().size());
	}
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(1, 2);

		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(1, paths.size());
		EXPECT_EQ(3, paths.front().size());
	}

}

TEST_F(CoordGraphTest, findShortestPath)
{
	//  0--1--2
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);

		EXPECT_EQ(CoordGraph::Path({ 0, 1, 2 }), g.findShortestPath(0, 2));
	}

	//  0--1--2--3--4
	//	 \		   /
	//	  5-------6
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 3);
		g.addEdge(3, 4);
		g.addEdge(0, 5);
		g.addEdge(5, 6);
		g.addEdge(6, 4);

		EXPECT_EQ(CoordGraph::Path({ 0, 5, 6, 4 }), g.findShortestPath(0, 4));
		EXPECT_EQ(CoordGraph::Path({ 4, 6, 5, 0 }), g.findShortestPath(4, 0));
	}
	//  0--1--3--4
	//	 \		/
	//	  5----6
	{
		CoordGraph g;
		g.addEdge(0, 5);
		g.addEdge(5, 6);
		g.addEdge(6, 4);
		g.addEdge(0, 1);
		g.addEdge(1, 3);
		g.addEdge(3, 4);
		EXPECT_EQ(CoordGraph::Path({ 0, 5, 6, 4 }), g.findShortestPath(0, 4));
	}
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 3);
		g.addEdge(3, 4);
		g.addEdge(0, 5);
		g.addEdge(5, 6);
		g.addEdge(6, 4);
		EXPECT_EQ(CoordGraph::Path({ 0, 1, 3, 4 }), g.findShortestPath(0, 4));
	}

}

TEST_F(CoordGraphTest, findShortestPath_noPath)
{
	// 0 - 1 - 2
	//
	// 3 - 4 - 5

	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(3, 4);
	g.addEdge(4, 5);

	EXPECT_EQ(CoordGraph::Path({}), g.findShortestPath(0, 3));
}



TEST_F(CoordGraphTest, findContours) 
{
	{
		std::vector<Element> elems;
		{
			Element segment;
			segment.vertices = { 0,1 };
			elems.push_back(segment);
			segment.vertices = { 1,2 };
			elems.push_back(segment);
			segment.vertices = { 2,3 };
			elems.push_back(segment);
		}

		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}

		CoordGraph g(elemPtrs);
		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(1, paths.size());
		EXPECT_EQ(4, paths.front().size());
	}

	{
		std::vector<Element> elems;
		{
			Element segment;
			segment.vertices = { 0,1 };
			elems.push_back(segment);
			segment.vertices = { 2,3 };
			elems.push_back(segment);
			segment.vertices = { 1,2 };
			elems.push_back(segment);
		}
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}

		CoordGraph g(elemPtrs);
		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(1, paths.size());
		EXPECT_EQ(4, paths.front().size());
	}

	{
		std::vector<Element> elems;
		{
			Element segment;
			segment.vertices = { 0,1 };
			elems.push_back(segment);
			segment.vertices = { 1,2 };
			elems.push_back(segment);
			segment.vertices = { 2,3 };
			elems.push_back(segment);

			segment.vertices = { 4,5 };
			elems.push_back(segment);
			segment.vertices = { 5,6 };
			elems.push_back(segment);
			segment.vertices = { 6,7 };
			elems.push_back(segment);
		}
		std::vector<const Element*> elemPtrs;
		for (auto const& elem : elems) {
			elemPtrs.push_back(&elem);
		}

		CoordGraph g(elemPtrs);
		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(2, paths.size());
		EXPECT_EQ(4, paths.front().size());
		EXPECT_EQ(4, paths.back().size());
	}

}

TEST_F(CoordGraphTest, alternativeAcyclicPahts) 
{

	// 13
	// | 
	// 11------12 -- 15
	// |     / |
	// |    /  |
	// |   /   |
	// |  / 16 |
	// | /    \|
	// 10------14 -- 17
	std::vector<Element> elems;
	{
		Element triangle;
		triangle.vertices = { 10, 11, 12 };
		elems.push_back(triangle);
		triangle.vertices = { 10, 12, 14 };
		elems.push_back(triangle);
	}


	CoordGraph g = CoordGraph(getElementsView(elems));
	g.addEdge(11, 13);
	g.addEdge(12, 15);
	g.addEdge(14, 16);
	g.addEdge(14, 17);
	CoordGraph gNoPaths = CoordGraph(getElementsView(elems));

	//auto cycles = g.findOrientedCycles();
	//ASSERT_EQ(3, cycles.size());
	//auto paths = g.findAcyclicPaths();
	//ASSERT_EQ(4, paths.size());
	//for (const auto& elem : elems){
	//	IdSet cIdSet;
	//	for (const auto& cId : elem.vertices) {
	//		cIdSet.insert(cId);
	//	}
	//	CoordGraph intersect = g.intersect(cIdSet);
	//	for (const auto& oCycle : intersect.findOrientedCycles()) {
	//		for (std::size_t vertex = 0; vertex < oCycle.size(); vertex++) {
	//			const CoordinateId edgeStart = oCycle[vertex];
	//			const CoordinateId edgeEnd = oCycle[(vertex + 1) % oCycle.size()];
	//			g.removeEdge(edgeStart, edgeEnd);
	//		}
	//	}

	//}
	CoordGraph diff = g.difference(gNoPaths);

	ASSERT_EQ(0, diff.findCycles().size());
	ASSERT_EQ(3, diff.findAcyclicPaths().size());

}

TEST_F(CoordGraphTest, getAcyclicEdges)
{
	// 11 <-- 12
	//   <
	//    \
	//    14
	CoordGraph g;
	g.addEdge(12, 11);
	g.addEdge(14, 11);

	auto a{ g.getAcyclicEdges() };
	ASSERT_EQ(2, a.size());
	EXPECT_EQ(CoordGraph::EdgeIds({ 12, 11 }), *a.begin());
}

TEST_F(CoordGraphTest, getAcyclicEdges_2)
{
	// 1 -> 2 -> 3
	//       \ 
	//        > 4

	CoordGraph g;
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(2, 4);

	EXPECT_EQ(3, g.getAcyclicEdges().size());
}

TEST_F(CoordGraphTest, getAcyclicEdges_3)
{
	// 10 -> 11 -> 12 -> 15
	//        \   /
	//        >  <
	//         14

	CoordGraph g;
	g.addEdge(10, 11);
	g.addEdge(11, 12);
	g.addEdge(12, 14);
	g.addEdge(14, 11);
	g.addEdge(12, 15);

	EXPECT_EQ(2, g.getAcyclicEdges().size());
}

TEST_F(CoordGraphTest, findAcyclicPaths)
{
	// 11 -- 12
	//   \
	//    14
	CoordGraph g;
	g.addEdge(12, 11);
	g.addEdge(14, 11);
	
	ASSERT_EQ(0, g.findCycles().size());
	ASSERT_EQ(1, g.findAcyclicPaths().size());
}

TEST_F(CoordGraphTest, findAcyclicPaths_2)
{
	// 1 -- 2 -- 3
	//       \ 
	//       4
	
	CoordGraph g;
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(2, 4);

	auto paths = g.findAcyclicPaths();
	EXPECT_EQ(2, paths.size());
}

TEST_F(CoordGraphTest, findAcyclicPaths_3)
{
	// 10 -- 11 -- 12 -- 15
	//        \   /
	//          14
	CoordGraph g;
	g.addEdge(10, 11);
	g.addEdge(11, 12);
	g.addEdge(12, 14);
	g.addEdge(14, 11);
	g.addEdge(12, 15);

	auto cycles = g.findCycles();
	ASSERT_EQ(1, cycles.size());

	auto paths = g.findAcyclicPaths();
	ASSERT_EQ(2, paths.size());
}

TEST_F(CoordGraphTest, findCycles_findAcyclicPaths) 
{
	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		
		auto cycles = g.findCycles();
		EXPECT_EQ(0, cycles.size());
		
		auto paths = g.findAcyclicPaths();
		ASSERT_EQ(1, paths.size());
		EXPECT_EQ(3, paths.front().size());
	}

	{
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 0);

		auto cycles = g.findCycles();
		ASSERT_EQ(1, cycles.size());
		EXPECT_EQ(3, cycles.front().size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(0, paths.size());
	}

	{
		// 0 - 1
		// | \ |
		// |   2
		// 3
		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 0);
		g.addEdge(0, 3);

		auto cycles = g.findCycles();
		ASSERT_EQ(1, cycles.size());
		EXPECT_EQ(3, cycles.front().size());

		auto paths = g.findAcyclicPaths();
		ASSERT_EQ(1, paths.size());
		EXPECT_EQ(2, paths.front().size());
	}

	{
		//   <---------
		// 0 --> 1  --> 2
		//  \> 3  <---

		CoordGraph g;
		g.addEdge(0, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 0);
		g.addEdge(0, 3);
		g.addEdge(2, 3);

		auto cycles = g.findCycles();
		EXPECT_EQ(1, cycles.size());

		auto paths = g.findAcyclicPaths();
		EXPECT_EQ(0, paths.size());
	}
}

TEST_F(CoordGraphTest, findCycles_performance_1)
{
	//       > 500
	//       | </
	// 0 --> 1 --> 2 --> .... --> 100
	// ^_________________________/
	
	CoordGraph g;
	std::size_t N = 100;
	for (std::size_t i = 0; i < N; i++) {
		g.addEdge(i, (i + 1) % N);
	}
	g.addEdge(1, 500);
	g.addEdge(500, 1);

	EXPECT_EQ(2, g.findCycles().size());
}

TEST_F(CoordGraphTest, findCycles_performance_2)
{
	CoordGraph g;
	std::size_t N = 7;
	for (std::size_t i = 0; i < N; i++) {
		g.addEdge(i, (i + 1)%N);
		for (std::size_t j = 0; j < i; j++) {
			g.addEdge(j+1, j);
		}
	}

	auto cs = g.findCycles();
	EXPECT_EQ(7, cs.size());
}

TEST_F(CoordGraphTest, findCycles_oriented_graph)
{
	// 0->1->2
	//  <---
	CoordGraph g; 
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 0);

	ASSERT_EQ(1, g.findCycles().size());
	EXPECT_EQ(3, g.findCycles().front().size());

	EXPECT_TRUE(g.isOrientableAndCyclic(g.findCycles().front()));
}

TEST_F(CoordGraphTest, findCycles)
{
	{
		//5 ---->---- 6
		//^			  v
		//2 ---->---- 3
		//^			  v
		//1 ----<---- 0
		//^			  v	
		//4 ----<---- 7

		CoordGraph g;
		g.addEdge(4, 1);
		g.addEdge(1, 2);
		g.addEdge(2, 5);
		g.addEdge(5, 6);
		g.addEdge(6, 3);
		g.addEdge(3, 0);
		g.addEdge(0, 7);
		g.addEdge(7, 4);
		g.addEdge(2, 3);
		g.addEdge(0, 1);
		EXPECT_EQ(4, g.findCycles().size());
	}
	{
		//5 ----<---- 6
		//v			  ^
		//2 ----<---- 3
		//v			  ^
		//1 ---->---- 0
		//v			  ^	
		//4 ---->---- 7

		CoordGraph g;
		g.addEdge(1, 4);
		g.addEdge(2, 1);
		g.addEdge(5, 2);
		g.addEdge(6, 5);
		g.addEdge(3, 6);
		g.addEdge(0, 3);
		g.addEdge(7, 0);
		g.addEdge(4, 7);
		g.addEdge(3, 2);
		g.addEdge(1, 0);
		EXPECT_EQ(4, g.findCycles().size());
	}
}

TEST_F(CoordGraphTest, findCycles_in_boundary_graph)
{

	// 24    4 
	// |  \  | \
	// 19 - 12 - 16 

	Elements es = {
		Element({ 24, 12, 19}, Element::Type::Surface),
		Element({ 12,  4, 16}, Element::Type::Surface)
	};

	auto cycles = CoordGraph(getElementsView(es)).getBoundaryGraph().findCycles();

	ASSERT_EQ(2, cycles.size());
	EXPECT_EQ(CoordGraph::Path({ 24, 12, 19 }), cycles[0]);
	EXPECT_EQ(CoordGraph::Path({ 12,  4, 16 }), cycles[1]);
}

TEST_F(CoordGraphTest, findCycles_non_oriented_graph_1)
{
	// 0->1->2
	//  <---
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(0, 2); // <- Non oriented

	EXPECT_EQ(0, g.findCycles().size());
}

TEST_F(CoordGraphTest, findCycles_non_oriented_graph_2)
{
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 1);// <- Non oriented
	g.addEdge(2, 3); 
	g.addEdge(3, 0);

	ASSERT_NO_THROW(g.findCycles());
	ASSERT_NO_THROW(g.findAcyclicPaths());
}


TEST_F(CoordGraphTest, findCycle_double_edge) 
{

	// 7 < < 2
	// v    || 
	// v    ||
	// v    ||
	// 4 > > 0

	CoordGraph g;
	g.addEdge(7, 4);
	g.addEdge(4, 0);
	g.addEdge(0, 2);
	g.addEdge(2, 7);
	g.addEdge(2, 0);

	EXPECT_EQ(2, g.findCycles().size());

}

TEST_F(CoordGraphTest, findCycle_double_edge_different_order_in_addition)
{

	// 7 < < 2
	// v    || 
	// v    ||
	// v    ||
	// 4 > > 0

	CoordGraph g;
	g.addEdge(7, 4);
	g.addEdge(4, 0);
	g.addEdge(0, 2);
	g.addEdge(2, 0);
	g.addEdge(2, 7);

	EXPECT_EQ(2, g.findCycles().size());
}

TEST_F(CoordGraphTest, findCycles_non_oriented_graph_3)
{
	CoordGraph g;
	g.addEdge(8, 24);
	g.addEdge(24, 72);
	g.addEdge(72, 8);
	g.addEdge(8, 130);
	g.addEdge(130, 131);
	g.addEdge(131, 72);

	ASSERT_EQ(2, g.findCycles().size());
	auto cycles = g.findCycles();
	EXPECT_TRUE(g.isOrientableAndCyclic(cycles[0]));
	EXPECT_TRUE(g.isOrientableAndCyclic(cycles[1]));

}

TEST_F(CoordGraphTest, findOrientedCycles_in_cell) 
{
	//  2 ----- 3
	//  |       |
	//  1 - 6 - 4 
	//  |       |
	//  0 ----- 5
	
	CoordGraph g;
	g.addEdge(0, 1);
	g.addEdge(1, 2);
	g.addEdge(2, 3);
	g.addEdge(3, 4);
	g.addEdge(4, 5);
	g.addEdge(5, 0);
    g.addEdge(1, 6);
	g.addEdge(6, 4);
		
	EXPECT_EQ(2, g.findCycles().size());

	g.addEdge(4, 6);
	g.addEdge(6, 1);

	EXPECT_EQ(5, g.findCycles().size());
}

TEST_F(CoordGraphTest, getExterior)
{
	// 57 ---- 49
	// | \     |
	// |   \   |
	// |     \ |
	// 58 ---- 50
	std::vector<Element> elems;
	{
		Element t;
		t.vertices = { 49, 50, 57 };    elems.push_back(t);
		t.vertices = { 57, 50, 58 };    elems.push_back(t);
	}
	
	EXPECT_THROW(CoordGraph(getElementsView(elems)).getExterior(), std::runtime_error);

}

TEST_F(CoordGraphTest, difference)
{

	// 57 ---- 49  57
	// | \     |  | \    
	// |   \   |  |   \   
	// |     \ |  |     \ 
	// 58 ---- 50  58 ---- 50

	CoordGraph g1;
	g1.addEdge(57, 50);
	g1.addEdge(50, 58);
	g1.addEdge(58, 57);
	g1.addEdge(57, 49);
	g1.addEdge(49, 50);
	g1.addEdge(50, 57);

	CoordGraph g2;
	g2.addEdge(57, 50);
	g2.addEdge(50, 58);
	g2.addEdge(58, 57);

	CoordGraph diff = g1.difference(g2);
	ASSERT_EQ(1, diff.findCycles().size());
	ASSERT_EQ(0, diff.findAcyclicPaths().size());
	IdSet ids{49,50,57};
	ASSERT_EQ( ids, diff.getVertices());

}

}
}