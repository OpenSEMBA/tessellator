#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "core/Slicer.h"

#include <map>
#include <set>

#if APP_LOADED
	#include "app/vtkIO.h"
#endif

namespace meshlib::core {
using namespace utils;
using namespace meshFixtures;
using namespace meshTools;

class SmootherTest : public ::testing::Test {
protected:
	const double sSAngle = 30.0;
	const double alignmentAngle = 5.0;
};

TEST_F(SmootherTest, self_intersecting_after_smoothing)
{
	// 2
	// | \
	// |  {3,4}
	// | / \
	// 0 -- 1
	
	Mesh m;
	{
		m.grid = meshFixtures::buildUnitLengthGrid(1.0);
		m.coordinates = {
			Coordinate({0.00, 0.00, 0.000}),
			Coordinate({1.00, 0.00, 0.000}),
			Coordinate({0.00, 1.00, 0.000}),
			Coordinate({0.50, 0.50, 0.000}),
			Coordinate({0.50, 0.50, 0.001}),
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 3, 1}),
			Element({3, 0, 2}),
			Element({0, 1, 4}),
			Element({0, 4, 2})
		};
	}

	auto r{ Smoother{m}.getMesh() }; 

	EXPECT_EQ(2, countMeshElementsIf(r, isTriangle));
}

TEST_F(SmootherTest, non_manifold)
{
	EXPECT_EQ(3, countMeshElementsIf(Smoother{ buildNonManifoldPatchMesh(1.0) }.getMesh(), isTriangle));
}

TEST_F(SmootherTest, touching_by_single_point)
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

	auto r{ Smoother{m}.getMesh() };

	EXPECT_EQ(1, countMeshElementsIf(r, isTriangle));
}

TEST_F(SmootherTest, retriangulatesMirroredPlanarPatchesWithMirroredDiagonals)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
	mesh.coordinates = {
		Relative({0.0, 0.0, 0.0}),
		Relative({0.0, 0.0, 1.0}),
		Relative({1.0, 1.0, 1.0}),
		Relative({1.0, 1.0, 0.0})
	};
	mesh.groups = {Group(), Group()};
	mesh.groups[0].elements = {
		Element({0, 3, 1}, Element::Type::Surface),
		Element({1, 3, 2}, Element::Type::Surface)
	};
	mesh.groups[1].elements = {
		Element({0, 2, 3}, Element::Type::Surface),
		Element({0, 1, 2}, Element::Type::Surface)
	};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);
	const auto internalEdge = [](const Elements& elements) {
		std::map<std::pair<CoordinateId, CoordinateId>, std::size_t> edgeUses;
		for (const auto& triangle : elements) {
			for (std::size_t vertex = 0; vertex < triangle.vertices.size(); ++vertex) {
				++edgeUses[std::minmax(
					triangle.vertices[vertex],
					triangle.vertices[(vertex + 1) % triangle.vertices.size()])];
			}
		}
		return std::find_if(
			edgeUses.begin(), edgeUses.end(),
			[](const auto& edgeUse) { return edgeUse.second == 2; })->first;
	};

	EXPECT_EQ((std::pair<CoordinateId, CoordinateId>{0, 2}),
		internalEdge(result.groups[0].elements));
	EXPECT_EQ((std::pair<CoordinateId, CoordinateId>{1, 3}),
		internalEdge(result.groups[1].elements));
	for (const auto& triangle : result.groups[0].elements) {
		EXPECT_GT(Geometry::normal(Geometry::asTriV(triangle, result.coordinates))[X], 0.0);
	}
	for (const auto& triangle : result.groups[1].elements) {
		EXPECT_LT(Geometry::normal(Geometry::asTriV(triangle, result.coordinates))[X], 0.0);
	}
}

TEST_F(SmootherTest, retriangulatesTrianglesAndPreservesNonTriangularElements)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
	mesh.coordinates = {
		Relative({0.0, 0.0, 0.0}),
		Relative({0.0, 0.0, 1.0}),
		Relative({1.0, 1.0, 1.0}),
		Relative({1.0, 1.0, 0.0})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {
		Element({0}, Element::Type::Node),
		Element({0, 1}, Element::Type::Line),
		Element({0, 1, 2, 3}, Element::Type::Surface),
		Element({0, 3, 1}, Element::Type::Surface),
		Element({1, 3, 2}, Element::Type::Surface)
	};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);

	ASSERT_EQ(result.groups[0].elements.size(), 5);
	EXPECT_EQ(result.groups[0].elements[0], mesh.groups[0].elements[0]);
	EXPECT_EQ(result.groups[0].elements[1], mesh.groups[0].elements[1]);
	EXPECT_EQ(result.groups[0].elements[2], mesh.groups[0].elements[2]);
	std::map<std::pair<CoordinateId, CoordinateId>, std::size_t> triangleEdgeUses;
	for (const auto& element : result.groups[0].elements) {
		if (!element.isTriangle()) {
			continue;
		}
		for (std::size_t vertex = 0; vertex < element.vertices.size(); ++vertex) {
			++triangleEdgeUses[std::minmax(
				element.vertices[vertex],
				element.vertices[(vertex + 1) % element.vertices.size()])];
		}
	}
	EXPECT_EQ((triangleEdgeUses[std::pair<CoordinateId, CoordinateId>{0, 2}]), 2);
	EXPECT_EQ((triangleEdgeUses[std::pair<CoordinateId, CoordinateId>{1, 3}]), 0);
}

TEST_F(SmootherTest, preservesConcavePlanarPatch)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
	mesh.coordinates = {
		Relative({0.1, 0.1, 0.5}), Relative({0.9, 0.1, 0.5}),
		Relative({0.9, 0.5, 0.5}), Relative({0.5, 0.5, 0.5}),
		Relative({0.5, 0.9, 0.5}), Relative({0.1, 0.9, 0.5})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {
		Element({0, 1, 3}), Element({1, 2, 3}),
		Element({0, 3, 5}), Element({3, 4, 5})
	};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);

	EXPECT_EQ(result.groups[0].elements, mesh.groups[0].elements);
}

TEST_F(SmootherTest, preservesDegenerateTriangles)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
	mesh.coordinates = {
		Relative({0.1, 0.1, 0.5}),
		Relative({0.5, 0.5, 0.5}),
		Relative({0.9, 0.9, 0.5})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {Element({0, 1, 2})};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);

	ASSERT_EQ(result.groups[0].elements.size(), 1);
	EXPECT_EQ(result.groups[0].elements[0], mesh.groups[0].elements[0]);
}

TEST_F(SmootherTest, preservesPlanarPatchWithMultipleBoundaryCycles)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
	mesh.coordinates = {
		Relative({0.1, 0.1, 0.5}), Relative({0.9, 0.1, 0.5}),
		Relative({0.9, 0.9, 0.5}), Relative({0.1, 0.9, 0.5}),
		Relative({0.35, 0.35, 0.5}), Relative({0.65, 0.35, 0.5}),
		Relative({0.65, 0.65, 0.5}), Relative({0.35, 0.65, 0.5})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {
		Element({0, 1, 5}), Element({0, 5, 4}),
		Element({1, 2, 6}), Element({1, 6, 5}),
		Element({2, 3, 7}), Element({2, 7, 6}),
		Element({3, 0, 4}), Element({3, 4, 7})
	};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);

	EXPECT_EQ(result.groups[0].elements, mesh.groups[0].elements);
}

TEST_F(SmootherTest, retriangulatesPlanarPatchesInAdjacentCellsIndependently)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 3);
	mesh.coordinates = {
		Relative({0.1, 0.1, 0.5}), Relative({0.1, 0.9, 0.5}),
		Relative({1.0, 0.9, 0.5}), Relative({1.0, 0.1, 0.5}),
		Relative({1.9, 0.9, 0.5}), Relative({1.9, 0.1, 0.5})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {
		Element({0, 3, 1}), Element({1, 3, 2}),
		Element({3, 5, 2}), Element({2, 5, 4})
	};

	const auto result = Smoother::retriangulatePlanarPatches(mesh);
	std::set<std::pair<CoordinateId, CoordinateId>> edges;
	for (const auto& triangle : result.groups[0].elements) {
		for (std::size_t vertex = 0; vertex < triangle.vertices.size(); ++vertex) {
			edges.insert(std::minmax(
				triangle.vertices[vertex],
				triangle.vertices[(vertex + 1) % triangle.vertices.size()]));
		}
	}

	EXPECT_EQ(edges.count({0, 2}), 1);
	EXPECT_EQ(edges.count({3, 4}), 1);
	EXPECT_EQ(edges.count({1, 3}), 0);
	EXPECT_EQ(edges.count({2, 5}), 0);
	EXPECT_NO_THROW(meshTools::checkNoCellsAreCrossed(result));
}

TEST_F(SmootherTest, rejectsElementsCrossingCellBoundaries)
{
	Mesh mesh;
	mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 3);
	mesh.coordinates = {
		Relative({0.5, 0.2, 0.5}),
		Relative({1.5, 0.2, 0.5}),
		Relative({0.5, 0.8, 0.5})
	};
	mesh.groups = {Group()};
	mesh.groups[0].elements = {Element({0, 1, 2})};

	EXPECT_THROW(
		Smoother::retriangulatePlanarPatches(mesh),
		std::runtime_error);
}

#if APP_LOADED

TEST_F(SmootherTest, preserves_topological_closedness_for_alhambra)
{
	
	auto m = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	
	m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
	
	auto slicedMesh = Slicer{m}.getMesh();
	
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh}.getMesh();
	
	EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(smoothedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.smoothed.vtk", smoothedMesh);
	// 
	// auto contourMesh = meshTools::buildMeshFromContours(smoothedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.contour.vtk", contourMesh);
}

TEST_F(SmootherTest, preserves_topological_closedness_for_sphere)
{
    auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    auto slicedMesh = Slicer{m}.getMesh();
    
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh, smootherOpts}.getMesh();

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);
	//
	// meshTools::convertToAbsoluteCoordinates(smoothedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.smoothed.vtk", smoothedMesh);
	//
	// auto contourMesh = meshTools::buildMeshFromContours(smoothedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}

#endif

}
