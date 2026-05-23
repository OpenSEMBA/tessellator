#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Smoother.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"
#include "core/Slicer.h"
#include "app/vtkIO.h"

namespace meshlib::core {
using namespace utils;
using namespace meshFixtures;
using namespace meshTools;

class SmootherTest : public ::testing::Test {
protected:
	const double sSAngle = 30.0;
	const double alignmentAngle = 5.0;

	static void assertCoordinatesListEquals(const Coordinates& expectedCoordinates, const Coordinates& resultCoordinates) {
		ASSERT_EQ(expectedCoordinates.size(), resultCoordinates.size());

		for (CoordinateId c = 0; c < expectedCoordinates.size(); ++c) {
			auto& expectedCoordinate = expectedCoordinates[c];
			auto& resultCoordinate = resultCoordinates[c];

			for (Axis axis = X; axis <= Z; ++axis) {
				EXPECT_EQ(expectedCoordinate[axis], resultCoordinate[axis])
					<< "Current coordinate: #" << c << std::endl
					<< "Current Axis: #" << axis << std::endl;
			}
		}
	}

	static void assertElementsListEquals(const Elements& expectedElements, const Elements& resultElements, GroupId g = 0) {
		ASSERT_EQ(expectedElements.size(), resultElements.size());

		for (ElementId e = 0; e < expectedElements.size(); ++e) {
			const Element& expectedElement = expectedElements[e];
			const Element& resultElement = resultElements[e];

			EXPECT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
				<< "Current Group: #" << g << std::endl
				<< "Current Element: #" << e << std::endl;
			EXPECT_EQ(resultElement.type, expectedElement.type)
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

	static Mesh buildMeshWithInnerDent()
	{
		//  z					 |   	z                         |	                    z       |	                   z
		// 10──────────────=9	 |   	9──────────────=16        |    16=──────────────17      |     17──────────────=10
		//  │          _-‾⟋ │	 |   	│           _-‾ │         |	    │ ⟍‾-_          │		|	   │           _-‾ │
		//  │       _-‾ ⟋   │	 |   	│       __-‾    │         |	    │   ⟍ ‾-_       │		|      │       __-‾    │
		//  │    _-‾ ⟋      │	 |   	│    _-‾        │         |	    │      ⟍ ‾-_    │		|      │    _-‾        │
		//  │ _-‾ ⟋         │	 |   	│ _-‾           │         |     │         ⟍ ‾-_ │		|      │ _-‾           │
		// 4,7═3,6═════════5,8	 |    3,5,8══════════-13,15       |    15──────────13───14      |     14=-════════════4,7
		//  │ _-‾-_				 |   	│       ___--‾‾ │         |	             _-‾ \  │       |      │       ___--‾‾ │
		//  2‾‾--__‾-_			 |     	│ __--‾‾        │         |	          _-‾     ‾\│       |      │ __--‾‾____----2
		//  0──────====1     x   |   	1=──────────────12	 y    |    x    12──────────11      |  y  11=======────────0
		//   					 |   	                          |	                            |	      
		//   					 |   	                          |	                            |
		// -------------------------------------------------------------------------------------------------------------------------
		//    					 |                                |	                     		|
		//  0────3─────1────5  x |						          |     9───────────────10	    |
		//  │    .    /│  / │    |                                |  	│ ⟍	            │		|
		//  │    .   / │ /  │    |                                |     │   ⟍ 	        │		|			 7___6____ 
		//  │    . /   │/   │    |                                |  	│     ⟍	        │		|			 |‾‾‾-----==.
		//  │    ./   .│    │    |                                |     │       ⟍   	│		|			 4 ----------8
		//  │   /.   . │    │    |                                |     │         ⟍ 	│		|			  \‾‾‾---___ |
		//  │ /  . .   │    │    |								  |     │           ⟍	│	    |			   \3------==5
		//  │/   ..	   │    │    |								  |     │             ⟍ ⎸	    |
		// 11────13────12───15   |						          |  x 16───────────────17	    |
		//	y					 |								  | 					y		|
		//						 |								  | 					 		|

		Mesh res;
		res.grid = utils::GridTools::buildCartesianGrid(0.0, 2.0, 3);
		res.coordinates = {
			Coordinate({ 0.00, 0.00, 0.00 }), // 0
			Coordinate({ 0.60, 0.00, 0.00 }), // 1
			Coordinate({ 0.00, 0.00, 0.10 }), // 2
			Coordinate({ 0.20, 0.00, 0.35 }), // 3
			Coordinate({ 0.00, 0.00, 0.40 }), // 4
			Coordinate({ 1.00, 0.00, 0.35 }), // 5
			Coordinate({ 0.20, 0.00, 0.45 }), // 6
			Coordinate({ 0.00, 0.00, 0.45 }), // 7
			Coordinate({ 1.00, 0.00, 0.40 }), // 8
			Coordinate({ 1.00, 0.00, 1.00 }), // 9
			Coordinate({ 0.00, 0.00, 1.00 }), // 10
			Coordinate({ 0.00, 1.00, 0.00 }), // 11
			Coordinate({ 0.60, 1.00, 0.00 }), // 12
			Coordinate({ 0.20, 1.00, 0.35 }), // 13
			Coordinate({ 0.00, 1.00, 0.40 }), // 14
			Coordinate({ 1.00, 1.00, 0.35 }), // 15
			Coordinate({ 1.00, 1.00, 1.00 }), // 16
			Coordinate({ 0.00, 1.00, 1.00 }), // 17
		};

		res.groups.push_back(Group());
		res.groups[0].elements = {
			Element({ 0, 1, 2 }, Element::Type::Surface), //	0 - 
			Element({ 1, 3, 2 }, Element::Type::Surface), //	1 - 
			Element({ 2, 3, 4 }, Element::Type::Surface), //	2 - 
			Element({ 3, 5, 4 }, Element::Type::Surface), //	3 - 
			Element({ 4, 5, 8 }, Element::Type::Surface), //	4 - 
			Element({ 4, 8, 7 }, Element::Type::Surface), //	5 - 
			Element({ 6, 7, 8 }, Element::Type::Surface), //	6 - 
			Element({ 6, 8, 9 }, Element::Type::Surface), //	7 -
			Element({ 6, 9, 7 }, Element::Type::Surface), //	8 -
			Element({ 7, 9, 10 }, Element::Type::Surface), //	9 -

			Element({ 11, 13, 12 }, Element::Type::Surface), // 10 -
			Element({ 11, 14, 13 }, Element::Type::Surface), // 11 - 
			Element({ 13, 14, 16 }, Element::Type::Surface), // 12 -
			Element({ 13, 16, 15 }, Element::Type::Surface), // 13 -
			Element({ 14, 17, 16 }, Element::Type::Surface), // 14 -


			Element({ 1, 12, 13 }, Element::Type::Surface), //	15 -
			Element({ 1, 13, 3 }, Element::Type::Surface), //	16 -
			Element({ 5, 15, 8 }, Element::Type::Surface), //	17 -
			Element({ 8, 15, 16 }, Element::Type::Surface), //	18 - 
			Element({ 8, 16, 9 }, Element::Type::Surface), //	19 -

			Element({ 0, 2, 11 }, Element::Type::Surface), //	20 - 
			Element({ 2, 4, 11 }, Element::Type::Surface), //	21 - 
			Element({ 4, 14, 11 }, Element::Type::Surface), //	22 -
			Element({ 4, 7, 14 }, Element::Type::Surface), //	23 - 
			Element({ 7, 10, 14 }, Element::Type::Surface), //	24 - 
			Element({ 10, 17, 14 }, Element::Type::Surface), // 25 - 


			Element({ 0, 11, 1 }, Element::Type::Surface), //	26 -
			Element({ 1, 11, 12 }, Element::Type::Surface), //	27 -
			Element({ 3, 13, 5 }, Element::Type::Surface), //	28 -
			Element({ 5, 13, 15 }, Element::Type::Surface), //	29 -

			Element({ 9, 16, 17 }, Element::Type::Surface), //	30 -
			Element({ 9, 17, 10 }, Element::Type::Surface), //	31 -
		};

		return res;
	}
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

///     z                               z
///     10──────────────=9             (10->5)──────────(9->4)
///      │          _-‾⟋ │	              │\            ⟋ │
///      │       _-‾ ⟋   │	              │ \         ⟋   │
///      │    _-‾ ⟋      │	              │  \     ⟋      │
///      │ _-‾ ⟋         │      ->	      │   \ ⟋         │
///    4 ,7═3,6═════════5,8               │ (3->2)──────(5->3)
///      │ _-‾-_                          │ _/‾-_
///      2‾‾--__‾-_                       │/     ‾-_
///      0──────====1     x               0─────────=1     x
///
/// ------------------------------------------------------------------
///                      z                            z
///     16=──────────────17          (16->10)─────────(17->11)
///      │ ⟍‾-_          │	              │ ⟍	         /│
///      │   ⟍ ‾-_       │	              │   ⟍	        / │
///      │      ⟍ ‾-_    │	              │      ⟍	   /  │
///      │         ⟍ ‾-_ │	    ->	      │         ⟍ /	  │
///     15──────────13───14            (15->9)────(13->8) │
///               _-‾ \  │                         _-‾ \  │
///            _-‾     ‾\│                      _-‾     ‾\│
///     x    12──────────11              x  (12->7)────(11->6)
///
/// --------------------------------------------------------
///      z                                z
///      9──────────────=16             (9->4)─────────(16->10)
///      │           _-‾ │                │ ‾-_           │
///      │       __-‾    │                │    ‾-__       │
///      │    _-‾        │      ->        │        ‾-_    │
///      │ _-‾           │                │           ‾-_ │
///    3,5,8══════════-13,15     (3->2),(5->3)─────────(13->8),(15->9)
///      │       ___--‾‾ │                │ ‾‾--___       │
///      │ __--‾‾        │                │        ‾‾--__ │
///      1=──────────────12	 y            1=───────────(12->7)	 y
///
/// ---------------------------------------------------------
///	                     z          	                   z
///     17──────────────=10           (17->11)─────────(10->5)
///	     │           _-‾ │          	  │             ⟋ │
///      │       __-‾    │                │           ⟋   │
///      │    _-‾        │                │         ⟋     │
///      │ _-‾           │      ->        │       ⟋       │
///     14=-════════════4,7               │     ⟋         │
///      │       ___--‾‾ │                │   ⟋           │
///      │ __--‾‾____----2                │ ⟋             │
///  y  11=======────────0           y (11->6)────────────0
///
/// ---------------------------------------------------------
///	                                	                   z
///      0────3─────1────5  x             0─(3->2)───1──(5->3)
///	     │    .    /│  / │                │    .    /│  / │
///      │    .   / │ /  │                │    .   / │ /  │
///      │    . /   │/   │                │    . /   │/   │
///      │    ./   .│    │      ->        │    ./   .│    │
///      │   /.   . │    │                │   /.   . │    │
///      │ /  . .   │    │                │ /  . .   │    │
///      │/   ..    │    │                │/   ..    │    │
///     11────13────12───15          (11->6)───*─(12->7)─(15->9)
///                                         (13->8)
///											
/// --------------------------------------------------------------------
///
///      9───────────────10             (9->4)─────────(10->5)
///   	 │ ⟍	         │             	  │             ⟋ ⎸
///      │   ⟍ 	         │                │           ⟋	  │
///   	 │     ⟍	     │             	  │         ⟋	  │
///      │       ⟍   	 │      ->        │       ⟋	      │
///      │         ⟍ 	 │                │     ⟋		  │
///      │           ⟍	 │                │   ⟋	          │
///      │             ⟍ ⎸				  │ ⟋	          │
///   x 16───────────────17        x  (16->10)─────────(17->11)
///  					y             					y
///

TEST_F(SmootherTest, smoothPointscontour)
{
	Mesh mesh = buildMeshWithInnerDent();
	const Elements& elements = mesh.groups[0].elements;

	SmootherOptions smootherOpts;
	smootherOpts.featureDetectionAngle = 30;
	smootherOpts.contourAlignmentAngle = 0;

	ASSERT_TRUE(meshTools::isAClosedTopology(mesh.groups[0].elements));

	Coordinates expectedCoordinates = {
			Coordinate({ 0.00, 0.00, 0.00 }),
			Coordinate({ 0.60, 0.00, 0.00 }),
			Coordinate({ 0.20, 0.00, 0.35 }),
			Coordinate({ 0.00, 0.00, 0.40 }),
			Coordinate({ 1.00, 0.00, 0.35 }),
			Coordinate({ 0.00, 0.00, 1.00 }),
			Coordinate({ 1.00, 0.00, 1.00 }),
			Coordinate({ 0.00, 1.00, 0.00 }),
			Coordinate({ 0.60, 1.00, 0.00 }),
			Coordinate({ 0.20, 1.00, 0.35 }),
			Coordinate({ 1.00, 1.00, 0.35 }),
			Coordinate({ 1.00, 1.00, 1.00 }),
			Coordinate({ 0.00, 1.00, 1.00 }),
	};

	Elements expectedElements = {
		Element({  5,  2,  6 }, Element::Type::Surface),
		Element({  3,  0,  2 }, Element::Type::Surface),
		Element({  4,  6,  2 }, Element::Type::Surface),
		Element({  2,  0,  1 }, Element::Type::Surface),
		Element({  5,  3,  2 }, Element::Type::Surface),
		Element({  7,  9,  8 }, Element::Type::Surface),
		Element({  9,  7, 11 }, Element::Type::Surface),
		Element({  9, 11, 10 }, Element::Type::Surface),
		Element({  7, 12, 11 }, Element::Type::Surface),
		Element({  1,  8,  9 }, Element::Type::Surface),
		Element({  1,  9,  2 }, Element::Type::Surface),
		Element({  4, 10, 11 }, Element::Type::Surface),
		Element({  4, 11,  6 }, Element::Type::Surface),
		Element({  0,  3,  7 }, Element::Type::Surface),
		Element({  3,  5,  7 }, Element::Type::Surface),
		Element({  5, 12,  7 }, Element::Type::Surface),
		Element({  0,  7,  1 }, Element::Type::Surface),
		Element({  1,  7,  8 }, Element::Type::Surface),
		Element({  2,  9,  4 }, Element::Type::Surface),
		Element({  4,  9, 10 }, Element::Type::Surface),
		Element({  6, 11, 12 }, Element::Type::Surface),
		Element({  6, 12,  5 }, Element::Type::Surface),
	};

	Mesh result = Smoother(mesh, smootherOpts).getMesh();

	assertCoordinatesListEquals(expectedCoordinates, result.coordinates);
	assertElementsListEquals(expectedElements, result.groups[0].elements);
}

TEST_F(SmootherTest, preserves_topological_closedness_for_alhambra)
{
	
	auto m = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	
	/**/

	m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
	
	/*

	m.grid[X] = utils::GridTools::linspace(42.0, 54.0, 7);
	m.grid[Y] = utils::GridTools::linspace(-52.0, -44.0, 5);
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);


	/*

	m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 16);
	m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 16);
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 2);

	/*
	
	m.grid[X] = utils::GridTools::linspace(-60.0, -36.0, 4);
	m.grid[Y] = utils::GridTools::linspace(-52.0, -28.0, 4);
	m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 2);

	/**/

	auto slicedMesh = Slicer{m}.getMesh();
	
	SmootherOptions smootherOpts;
    smootherOpts.featureDetectionAngle = 30;
    smootherOpts.contourAlignmentAngle = 0;
	auto smoothedMesh = Smoother{slicedMesh}.getMesh();
	
	EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
	EXPECT_TRUE(meshTools::isAClosedTopology(smoothedMesh.groups[0].elements));

    //For debugging.
	meshTools::convertToAbsoluteCoordinates(slicedMesh);
	meshTools::convertToAbsoluteCoordinates(smoothedMesh);


	vtkIO::exportGridToVTU("testData/cases/alhambra/alhambra.grid.vtk", smoothedMesh.grid);
	vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.sliced.vtk", slicedMesh);
	vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.smoothed.vtk", smoothedMesh);
	 
	auto contourMesh = meshTools::buildMeshFromContours(smoothedMesh);
	vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.contour.vtk", contourMesh);
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

}