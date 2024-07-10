#include "gtest/gtest.h"

#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "MeshFixtures.h"

using namespace meshlib;
using namespace tessellator;
using namespace utils;
using namespace meshFixtures;

class SmootherToolsTest : public ::testing::Test {
protected:
	static Coordinates buildSimplePatchCoordinates() {
		return Coordinates({
			Coordinate({2.48, 3.30, 2.20}),
			Coordinate({2.57, 2.88, 2.20}),
			Coordinate({2.00, 3.78, 2.20}),
			Coordinate({2.35, 3.10, 2.20}),
			Coordinate({2.22, 2.74, 2.20}),
			Coordinate({2.31, 3.00, 2.20}),
			Coordinate({2.45, 3.00, 2.20}),
			Coordinate({2.16, 3.00, 2.20}),
			Coordinate({2.55, 3.00, 2.20})
		});
	}

	static Mesh buildTwoCellsPatchMesh()
	{
		//   2 ----- 0
		//  / \    /  \
		//  7 =  5 === 8
		//  |  /  \   /
		//  | /    \ /
		//  4 ----- 1
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
		res.coordinates = buildSimplePatchCoordinates();

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 2, 5, 7 }, Element::Type::Surface),
			Element({ 2, 0, 5 }, Element::Type::Surface),
			Element({ 0, 8, 5 }, Element::Type::Surface),
			Element({ 7, 5, 4 }, Element::Type::Surface),
			Element({ 4, 5, 1 }, Element::Type::Surface),
			Element({ 5, 8, 1 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildOnePointInCellFacePatchMesh() 
	{
		//   2 ----- 0
		//  / \    /  \
		//  7 =  5 === 8
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
		res.coordinates = buildSimplePatchCoordinates();
	
		Elements tris;
		{
			Element tri;
			tri.type = Element::Type::Surface;

			tri.vertices = { 2, 5, 7 }; tris.push_back(tri);
			tri.vertices = { 2, 0, 5 }; tris.push_back(tri);
			tri.vertices = { 0, 8, 5 }; tris.push_back(tri);
		}
		res.groups.push_back(Group());
		res.groups[0].elements = tris;
		
		return res;
	}

	static Mesh buildTwoPointsInCellFacePatchMesh()
	{
		//   2  ---- 0
		//  \  \   / | \
		//   7 = 5 = 6 = 8
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
		res.coordinates = buildSimplePatchCoordinates();

		Elements tris;
		{
			Element tri;
			tri.type = Element::Type::Surface;

			tri.vertices = { 2, 5, 7 }; tris.push_back(tri);
			tri.vertices = { 2, 0, 5 }; tris.push_back(tri);
			tri.vertices = { 0, 6, 5 }; tris.push_back(tri);
			tri.vertices = { 0, 8, 6 }; tris.push_back(tri);
		}
		res.groups.push_back(Group());
		res.groups[0].elements = tris;

		return res;
	}

	static Mesh buildInnerPointPatchMesh()
	{
		//   2   -----    0
		//   \   \   / /  | \
		//    \    3   |  |  \
 		//     \  / \  /  |   \
		//      7 === 5 = 6 == 8 
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
		res.coordinates = buildSimplePatchCoordinates();

		Elements tris;
		{
			Element tri;
			tri.type = Element::Type::Surface;
			tri.vertices = { 2, 3, 7 }; tris.push_back(tri);
			tri.vertices = { 2, 0, 3 }; tris.push_back(tri);
			tri.vertices = { 3, 0, 5 }; tris.push_back(tri);
			tri.vertices = { 3, 5, 7 }; tris.push_back(tri);
			tri.vertices = { 0, 6, 5 }; tris.push_back(tri);
			tri.vertices = { 0, 8, 6 }; tris.push_back(tri);
		}
		res.groups.push_back(Group());
		res.groups[0].elements = tris;

		return res;
	}

	static Mesh buildInnerPointPatchMesh2()
	{
		//   1 -----2
		//   |\\   /|
 		//   | |\ / |
		//   | | 5  |
		//   | |/  \|
		//   0-4----3 
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.00, 0.00, 0.10}),
			Coordinate({0.00, 1.00, 0.10}),
			Coordinate({1.00, 1.00, 0.10}),
			Coordinate({1.00, 0.00, 0.10}),
			Coordinate({0.05, 0.00, 0.10}),
			Coordinate({0.15, 0.15, 0.10})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 4 }, Element::Type::Surface),
			Element({ 1, 5, 4 }, Element::Type::Surface),
			Element({ 1, 2, 5 }, Element::Type::Surface),
			Element({ 2, 3, 5 }, Element::Type::Surface),
			Element({ 3, 4, 5 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildTwoInnerPointsPatchMesh()
	{
		//   1 ---- 2 
		//   | \ / /|
		//   |  4-5 |
		//   | / \ \|
		//   0 -----3
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.00, 0.00, 0.10}),
			Coordinate({0.00, 1.00, 0.10}),
			Coordinate({1.00, 1.00, 0.10}),
			Coordinate({1.00, 0.00, 0.10}),
			Coordinate({0.25, 0.50, 0.10}),
			Coordinate({0.75, 0.55, 0.10})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 4 }, Element::Type::Surface),
			Element({ 0, 4, 3 }, Element::Type::Surface),
			Element({ 1, 2, 4 }, Element::Type::Surface),
			Element({ 2, 5, 4 }, Element::Type::Surface),
			Element({ 2, 3, 5 }, Element::Type::Surface),
			Element({ 3, 4, 5 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildTwoInnerPointsPatchMeshNotConnected()
	{
		//   1 ---- 2 
		//   | \ / /|
		//   |  4\5 |
		//   | / \ \|
		//   0 -----3
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.00, 0.00, 0.10}),
			Coordinate({0.00, 1.00, 0.10}),
			Coordinate({1.00, 1.00, 0.10}),
			Coordinate({1.00, 0.00, 0.10}),
			Coordinate({0.25, 0.50, 0.10}),
			Coordinate({0.75, 0.55, 0.10})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 4 }, Element::Type::Surface),
			Element({ 0, 4, 3 }, Element::Type::Surface),
			Element({ 3, 4, 1 }, Element::Type::Surface),
			Element({ 3, 1, 5 }, Element::Type::Surface),
			Element({ 5, 1, 2 }, Element::Type::Surface),
			Element({ 3, 5, 2 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildThreeInnerPointsPatchMesh()
	{
		//   1 ---- 2 
		//   | \ / /|
		//   |  4-5 |
		//   | / 6 \|
		//   | // \ |
		//   0 -----3
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.00, 0.00, 0.10}),
			Coordinate({0.00, 1.00, 0.10}),
			Coordinate({1.00, 1.00, 0.10}),
			Coordinate({1.00, 0.00, 0.10}),
			Coordinate({0.25, 0.75, 0.10}),
			Coordinate({0.75, 0.75, 0.10}),
			Coordinate({0.50, 0.25, 0.10})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 4 }, Element::Type::Surface),
			Element({ 1, 2, 4 }, Element::Type::Surface),
			Element({ 2, 5, 4 }, Element::Type::Surface),
			Element({ 0, 4, 6 }, Element::Type::Surface),
			Element({ 4, 5, 6 }, Element::Type::Surface),
			Element({ 5, 2, 3 }, Element::Type::Surface),
			Element({ 6, 5, 3 }, Element::Type::Surface),
			Element({ 0, 6, 3 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildCollapseEdgeMesh()
	{
		//     5 --6-- 7
		//   / || \| / ||
		// 8 - 3 - 4   ||
		//   \ || /| \ ||
		//     0 == 1== 2
		//       \  |  /
		//          9
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 3);
		res.coordinates = {
			Coordinate({1.00, 1.00, 0.00}),
			Coordinate({1.50, 1.00, 0.00}),
			Coordinate({2.00, 1.00, 0.00}),
			Coordinate({1.00, 1.25, 0.00}),
			Coordinate({1.50, 1.25, 0.00}),
			Coordinate({1.00, 1.50, 0.00}),
			Coordinate({1.50, 1.50, 0.00}),
			Coordinate({2.00, 1.50, 0.00}),
			Coordinate({0.50, 1.25, 0.00}),
			Coordinate({1.50, 0.50, 0.00})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 4 }, Element::Type::Surface),
			Element({ 1, 2, 4 }, Element::Type::Surface),
			Element({ 0, 4, 3 }, Element::Type::Surface),
			Element({ 3, 4, 5 }, Element::Type::Surface),
			Element({ 4, 6, 5 }, Element::Type::Surface),
			Element({ 4, 7, 6 }, Element::Type::Surface),
			Element({ 2, 7, 4 }, Element::Type::Surface),
			Element({ 8, 0, 3 }, Element::Type::Surface),
			Element({ 8, 3, 5 }, Element::Type::Surface),
			Element({ 9, 1, 0 }, Element::Type::Surface),
			Element({ 9, 2, 1 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildElementsToRemesh()
	{
		//   4 ---- 5 
		//   | \_   |
		//   |    \ |
		//   2 ---- 3
		//   | \  / |
		//   |   6  |
		//   | /   \|
		//   0 ---- 1
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.00, 0.00, 0.10}),
			Coordinate({1.00, 0.00, 0.10}),
			Coordinate({0.00, 0.75, 0.10}),
			Coordinate({1.00, 0.75, 0.10}),
			Coordinate({0.00, 1.00, 0.10}),
			Coordinate({1.00, 1.00, 0.10}),
			Coordinate({0.50, 0.25, 0.10})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 6 }, Element::Type::Surface),
			Element({ 1, 3, 6 }, Element::Type::Surface),
			Element({ 6, 3, 2 }, Element::Type::Surface),
			Element({ 0, 6, 2 }, Element::Type::Surface),
			Element({ 2, 3, 4 }, Element::Type::Surface),
			Element({ 3, 5, 4 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildCornerMesh()
	{
		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
		res.coordinates = {
			Coordinate({0.50, 0.00, 0.50}),
			Coordinate({0.00, 0.00, 0.50}),
			Coordinate({0.00, 0.50, 0.50}),
			Coordinate({0.00, 0.50, 0.00}),
			Coordinate({0.50, 0.50, 0.00}),
			Coordinate({0.50, 0.00, 0.00}),
			Coordinate({0.50, 0.50, 0.50}),
			Coordinate({0.50, 0.25, 0.50})
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 7, 1 }, Element::Type::Surface),
			Element({ 7, 6, 1 }, Element::Type::Surface),
			Element({ 6, 2, 1 }, Element::Type::Surface),
			Element({ 2, 6, 4 }, Element::Type::Surface),
			Element({ 3, 2, 4 }, Element::Type::Surface),
			Element({ 5, 4, 6 }, Element::Type::Surface),
			Element({ 5, 6, 7 }, Element::Type::Surface),
			Element({ 5, 7, 0 }, Element::Type::Surface)
		};

		return res;
	}

	static Mesh buildLShapedMesh()
	{
		//  0 ------- 1
		//  | \ \_  / |
		//  2= 3 = 4 =5
		// || / _/  \ ||
		//  6 ========7
		Mesh m;
		m.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 2);

		m.coordinates = {
			Coordinate({0.00, 0.50, 0.50}),
			Coordinate({1.00, 0.50, 0.50}),
			Coordinate({0.00, 0.50, 0.00}), // Feature
			Coordinate({0.25, 0.50, 0.00}), // Feature
			Coordinate({0.75, 0.50, 0.00}), // Feature
			Coordinate({1.00, 0.50, 0.00}), // Feature
			Coordinate({0.00, 1.00, 0.00}),
			Coordinate({1.00, 1.00, 0.00})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 3, 2}),
			Element({0, 4, 3}),
			Element({0, 1, 4}),
			Element({1, 5, 4}),
			Element({2, 3, 6}),
			Element({3, 4, 6}),
			Element({4, 7, 6}),
			Element({4, 5, 7})
		};
		return m;
	}

	static std::size_t countDifferentCoordinates(const Coordinates& c)
	{
		return std::set<Coordinate>(c.begin(), c.end()).size();
	}

	static std::size_t countTriangles(const Elements& es)
	{
		Elements tris;
		std::copy_if(
			es.begin(), es.end(),
			std::back_inserter(tris),
			[](const Element& e) { return e.isTriangle(); }
		);
		return tris.size();
	}

	const double sSAngle = 30.0;
	const double alignmentAngle = 5.0;
};

/// Remesh patch with interior points to remove them. 
///	\verbatim
///   1 ---- 2 
///   | \ / /|
///   |  4-5 |
///   | / \ \|
///   0 -----3 \endverbatim
/// Expected result:
/// - CoordinateId 4 and 5 collapse to a boundary point. 
/// - The list of coordinates decreases by 2.
/// - The number of elements decreases to 2.
TEST_F(SmootherToolsTest, remeshElementsAvoidingInteriorPoints)
{
	Mesh m = buildTwoInnerPointsPatchMesh();

	SmootherTools sT(m.grid);
	Coordinates collapsed = m.coordinates;
	Elements elems = m.groups[0].elements;
	sT.remeshWithNoInteriorPoints(elems, m.coordinates, getView<Element>(elems));

	//EXPECT_EQ(6, countDifferentCoordinates(m.coordinates));
	//EXPECT_EQ(4, countDifferentCoordinates(collapsed));

	EXPECT_EQ(6, countTriangles(m.groups[0].elements));
	EXPECT_EQ(2, countTriangles(elems));
}

/// Remesh patch with interior points to remove them. 
///	\verbatim
///   1 ---- 2 
///   | \ / /|
///   |  4\5 |
///   | / \ \|
///   0 -----3 \endverbatim
/// Expected result:
/// - CoordinateId 4 and 5 collapse to a boundary point. 
/// - The list of coordinates decreases by 2.
/// - The number of elements decreases to 2.
TEST_F(SmootherToolsTest, remeshElementsAvoidingInteriorPoints_2)
{
	Mesh m = buildTwoInnerPointsPatchMeshNotConnected();

	SmootherTools sT(m.grid);
	Coordinates collapsed = m.coordinates;
	Elements elems = m.groups[0].elements;
	sT.remeshWithNoInteriorPoints(elems, m.coordinates, getView<Element>(elems));

	//EXPECT_EQ(6, countDifferentCoordinates(m.coordinates));
	//EXPECT_EQ(4, countDifferentCoordinates(collapsed));

	EXPECT_EQ(6, countTriangles(m.groups[0].elements));
	EXPECT_EQ(2, countTriangles(elems));
}

/// Collapse points on the edges (represented by double lines || =).
/// verbatim
///     5 --6-- 7
///   / || \| / ||
/// 8 - 3 - 4   ||
///   \ || /| \ ||
///     0 == 1== 2
///       \  |  /
///          9		\endverbatim
/// CoordinateId 5, 0, 3, 1, 2 are on edges.
/// Expected results:
/// - CoordinateId 3 and 1 collapse to points on the extreme of their edges.
/// - The number of coordinates decreases by 2.
TEST_F(SmootherToolsTest, collapseEdge)
{
	Mesh mesh = buildCollapseEdgeMesh();
	const Elements& es = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid);
	auto sIds = sT.buildSingularIds(es, mesh.coordinates, sSAngle);
		
	Coordinates collapsed = mesh.coordinates;
	for (auto const& c : sT.buildCellElemMap(es, mesh.coordinates)) {
		sT.collapsePointsOnCellEdges(collapsed, c.second, sIds, alignmentAngle);
	}
		
	EXPECT_EQ(std::set<Coordinate>(collapsed.begin(), collapsed.end()).size(), 8);
}

/// \verbatim
///   2 ----- 0
///  / \    /  \
///  7 =  5 === 8
///  |  /  \   /
///  | /    \ /
///  4 ----- 1	  \endverbatim
TEST_F(SmootherToolsTest, collapsePointsOnCellFaces)
{
	Mesh mesh = buildTwoCellsPatchMesh();
	const Elements& elems = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid);
	auto sIds = sT.buildSingularIds(elems, mesh.coordinates, sSAngle);

	Coordinates collapsed = mesh.coordinates;
	for (auto const c : sT.buildCellElemMap(elems, mesh.coordinates)) {
		sT.collapsePointsOnCellFaces(collapsed, c.second, sIds);
	}

	EXPECT_EQ(9, countDifferentCoordinates(mesh.coordinates));
	EXPECT_EQ(8, countDifferentCoordinates(collapsed));

	EXPECT_NE(collapsed[5], mesh.coordinates[5]);
}


/// Collapse the inner point with id 3 to the bound of the set of elements.
/// \verbatim
///   2   -----    0
///   \   \   / /  | \
///    \    3   |  |  \
///     \  / \  /  |   \
///      7 === 5 = 6 == 8  \endverbatim
/// Expected result:
/// - CoordinateId 3 is removed from the list of coordinates.
TEST_F(SmootherToolsTest, collapse_inner_point)
{
	Mesh mesh = buildInnerPointPatchMesh();
	const Elements& elems = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid);
	Coordinates collapsed = mesh.coordinates;
	sT.collapseInteriorPointsToBound(collapsed, getView<Element>(elems));

	EXPECT_EQ(9, countDifferentCoordinates(mesh.coordinates));
	EXPECT_EQ(8, countDifferentCoordinates(collapsed));

	EXPECT_NE(collapsed[3], mesh.coordinates[3]);
}

/// Collapse the inner point with id 5 to the bound of the set of elements.
///\verbatim
///   1 -----2
///   |\\   /|
///   | |\ / |
///   | | 5  |
///   | |/  \|
///   0-4----3 \endverbatim
/// Expected result:
/// - CoordinateId 5 is removed from the list of coordinates.
TEST_F(SmootherToolsTest, collapse_inner_point_2)
{
	const Mesh mesh = buildInnerPointPatchMesh2();
	const Elements& elems = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid);
	Coordinates collapsed = mesh.coordinates;
	sT.collapseInteriorPointsToBound(collapsed, getView<Element>(elems));

	EXPECT_EQ(6, countDifferentCoordinates(mesh.coordinates));
	EXPECT_EQ(5, countDifferentCoordinates(collapsed));

	EXPECT_EQ(collapsed[5], mesh.coordinates[4]);
}
/// Collapse the points in the contour.
/// \verbatim
///   2 ----- 0
///  / \    /  \
///  7 =  5 === 8 \endverbatim
/// Expected result:
/// - CoordinateId 5 is collapsed to CoordinateId 7, closer than CoordinateId 8.
TEST_F(SmootherToolsTest, collapse_one_point_in_contour)
{
	Mesh mesh = buildOnePointInCellFacePatchMesh();
	const Elements& elems = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid); 
	Coordinates collapsed = 
		sT.collapsePointsOnContour(elems, mesh.coordinates, alignmentAngle);
	
	EXPECT_EQ(9, countDifferentCoordinates(mesh.coordinates));
	EXPECT_EQ(8, countDifferentCoordinates(collapsed));
	EXPECT_EQ(collapsed[5], collapsed[7]);
}

/// Collapse points in the contour, when there is more than point to collapse.
/// \verbatim
///   2  ---- 0
///  \  \   / | \
///   7 = 5 = 6 = 8 \endverbatim
/// Expected result:
/// - CoordinateId 5 and 6 are removed from the list of coordinates.
/// - Coordinate Id 5 and 6 collapse to CoordinateId 7.
TEST_F(SmootherToolsTest, collapse_two_points_in_contour)
{
	Mesh mesh = buildTwoPointsInCellFacePatchMesh();
	const Elements& elems = mesh.groups[0].elements;

	SmootherTools sT(mesh.grid);
	Coordinates collapsed = 
		sT.collapsePointsOnContour(elems, mesh.coordinates, alignmentAngle);

	EXPECT_EQ(9, countDifferentCoordinates(mesh.coordinates));
	EXPECT_EQ(7, countDifferentCoordinates(collapsed));
	
	EXPECT_NE(collapsed[5], mesh.coordinates[5]);
	EXPECT_NE(collapsed[6], mesh.coordinates[6]);
	
	EXPECT_EQ(collapsed[5], collapsed[7]);
	EXPECT_EQ(collapsed[6], collapsed[7]);
}

/// Build the list of singular Ids corresponding to a corner structure with the following ids
/// \verbatim
///    2  -----  6
///   /|       7 |
///  / |      /  |
/// 1 ------ 0   |
///    |     |   |
///    3 --- |---4
///          |  /
///          | /  
///          5  \endverbatim
/// Expected result:
/// - Ids in contour: 0, 1, 2, 3, 4, 5
/// - Ids in features: 0, 2, 4, 6, 7 
/// - Ids in corner: 6

TEST_F(SmootherToolsTest, buildSingularIds)
{
	Mesh m = buildCornerMesh();
	const Coordinates& cs = m.coordinates;
	const Elements& es = m.groups[0].elements;


	SmootherTools sT(m.grid);
	auto sIds = sT.buildSingularIds(es, cs, sSAngle);

	EXPECT_EQ(IdSet({ 0, 1, 2, 3, 4, 5}), sIds.contourIds());
	EXPECT_EQ(IdSet({ 0, 2, 4, 6, 7 }), sIds.featureIds());
	EXPECT_EQ(IdSet({ 6 }), sIds.cornerIds());
	EXPECT_EQ(IdSet({ 0, 1, 2, 3, 4, 5, 6, 7 }), sIds.edgeIds());
}

TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_singlePatch)
{
	Mesh m = buildCornerMesh();
	Coordinates& cs = m.coordinates;
	Elements& es = m.groups[0].elements;

	SmootherTools sT(m.grid);
	auto sIds = sT.buildSingularIds(es, cs, sSAngle);
	
	ElementsView p({&es[0], &es[1], &es[2]});

	sT.collapsePointsOnFeatureEdges(cs, p, sIds);
	
	EXPECT_EQ(7, countDifferentCoordinates(cs));
}

TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_threePatches)
{
	Mesh m = buildCornerMesh();
	Coordinates& cs = m.coordinates;
	Elements& es = m.groups[0].elements;

	SmootherTools sT(m.grid);
	auto sIds = sT.buildSingularIds(es, cs, sSAngle);

	auto cells = sT.buildCellElemMap(es, cs);
	for (auto const& p : Geometry::buildDisjointSmoothSets(cells[Cell({ 0,0,0 })], cs, sSAngle)) {
		sT.collapsePointsOnFeatureEdges(cs, p, sIds);
	}

	EXPECT_EQ(7, countDifferentCoordinates(cs));
}


TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_interior_in_face)
{
	//    3   ||   0
	//  / | \ || / |\
	// 6  |    2   | 5
    //  \ | / || \ |/
	//    4   ||  1

	Mesh m;
	{
		m.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 2);

		m.coordinates = {
			Coordinate({1.8, 0.75, 0.2}),
			Coordinate({1.8, 0.25, 0.2}),
			Coordinate({1.0, 0.50, 0.2}),
			Coordinate({0.8, 0.75, 0.2}),
			Coordinate({0.8, 0.25, 0.2}),
			Coordinate({1.9, 0.50, 0.2}),
			Coordinate({0.1, 0.50, 0.2}),
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({2, 4, 3}),
			Element({0, 5, 1}),
			Element({6, 3, 4})
		};
	}


	SmootherTools::SingularIds sIds(
		{0, 1, 2}, // Feature
		{},        // Contour
		{}         // Corner
	);

	SmootherTools sT(m.grid);
	Coordinates collapsed = m.coordinates;
	for (auto const& cell : sT.buildCellElemMap(m.groups[0].elements, m.coordinates)) {
		for (auto const& p : Geometry::buildDisjointSmoothSets(cell.second, m.coordinates, sSAngle)) {
			sT.collapsePointsOnFeatureEdges(collapsed, p, sIds);
		}
	}
	m.coordinates = collapsed;
	EXPECT_EQ(7, countDifferentCoordinates(m.coordinates));
}

TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_interior_in_face_2)
{
	//    3   ||   0
	//  / | \ || / |\
	// 6  |    2   | 5
	//  \ | / || \ |/
	//    4 -  7 - 1

	Mesh m;
	{
		m.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 2);

		m.coordinates = {
			Coordinate({1.8, 0.75, 0.2}),
			Coordinate({1.8, 0.25, 0.2}),
			Coordinate({1.0, 0.50, 0.2}),
			Coordinate({0.8, 0.75, 0.2}),
			Coordinate({0.8, 0.25, 0.2}),
			Coordinate({1.9, 0.50, 0.2}),
			Coordinate({0.1, 0.50, 0.2}),
			Coordinate({1.0, 0.20, 0.2})
		};

		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2}),
			Element({2, 4, 3}),
			Element({0, 5, 1}),
			Element({6, 3, 4}),
			Element({4, 2, 7}),
			Element({7, 2, 1})
		};
	}

	SmootherTools::SingularIds sIds(
		{ 0, 1, 2, 7 }, // Feature
		{},        // Contour
		{}         // Corner
	);

	const Elements& es = m.groups[0].elements;
	Coordinates collapsed = m.coordinates;
	SmootherTools(m.grid)
		.collapsePointsOnFeatureEdges(collapsed, {&es[0], &es[2], &es[5]}, sIds);

	m.coordinates = collapsed;
	EXPECT_EQ(8, countDifferentCoordinates(m.coordinates));
}

TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_feature_over_face)
{
	//  0 ------- 1
	//  | \ \_  / |
	//  2= 3 = 4 =5
	// || / _/  \ ||
	//  6 ========7
	Mesh m = buildLShapedMesh();

	SmootherTools::SingularIds sIds(
		{ 2, 3, 4, 5 }, // Feature
		{},        // Contour
		{}         // Corner
	);

	const Elements& es = m.groups[0].elements;
	{
		Coordinates collapsed = m.coordinates;
		SmootherTools(m.grid)
			.collapsePointsOnFeatureEdges(collapsed, { &es[0], &es[1],  &es[2], &es[3] }, sIds);

		EXPECT_EQ(6, countDifferentCoordinates(collapsed));
		ASSERT_EQ(8, collapsed.size());
		EXPECT_EQ(collapsed[2], collapsed[3]);
		EXPECT_EQ(collapsed[5], collapsed[4]); 
	}
	{
		Coordinates collapsed = m.coordinates;
		SmootherTools(m.grid)
			.collapsePointsOnFeatureEdges(collapsed, { &es[4], &es[5],  &es[6], &es[7] }, sIds);

		EXPECT_EQ(6, countDifferentCoordinates(collapsed));
		ASSERT_EQ(8, collapsed.size());
		EXPECT_EQ(collapsed[2], collapsed[3]);
		EXPECT_EQ(collapsed[5], collapsed[4]);
	}
}

TEST_F(SmootherToolsTest, collapsePointsOnFeatureEdges_feature_in_interior)
{
	//  0 ------- 1
	//  | \ \_  / |
	//  2- 3 - 4 -5
	//  | / _/  \ |
	//  6 --------7
	Mesh m = buildLShapedMesh();
	std::transform(
		m.coordinates.begin(), m.coordinates.end(), 
		m.coordinates.begin(),
		[](auto const& c) {
			return c + Coordinate({ 0.0, 0.0, 0.25 });
		}
	);

	SmootherTools::SingularIds sIds(
		{ 2, 3, 4, 5 }, // Feature
		{},             // Contour
		{}              // Corner
	);

	const Elements& es = m.groups[0].elements;
	Coordinates collapsed = m.coordinates;
	SmootherTools(m.grid)
		.collapsePointsOnFeatureEdges(collapsed, { &es[0], &es[1],  &es[2], &es[3] }, sIds);

	EXPECT_EQ(6, countDifferentCoordinates(collapsed));
	ASSERT_EQ(8, collapsed.size());
	EXPECT_EQ(collapsed[2], collapsed[3]);
	EXPECT_EQ(collapsed[5], collapsed[4]);
}