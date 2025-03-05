#include "MeshFixtures.h"
#include "gtest/gtest.h"

#include "Slicer.h"
#include "Geometry.h"
#include "MeshTools.h"
#include "app/vtkIO.h"
#include "utils/RedundancyCleaner.h"

namespace meshlib::core {

using namespace meshFixtures;
using namespace utils;
using namespace meshTools;

class SlicerTest : public ::testing::Test {
public:
protected:
    const std::size_t X = 0;
    const std::size_t Y = 1;
    const std::size_t Z = 2;

    static bool containsDegenerateTriangles(const Mesh& out)
    {
        for (auto const& g : out.groups) {
            for (auto const& e : g.elements) {
                if (e.isTriangle() && 
                    Geometry::isDegenerate(Geometry::asTriV(e, out.coordinates))) {
                    return true;
                }
            }
        }
        return false;
    }
};



TEST_F(SlicerTest, buildTrianglesFromPath_1)
{
    Coordinates cs = {
        Coordinate({33.597101430000002, 0.0000000000000000,  2.0990656099999998}),
        Coordinate({34.000000000000000, 0.74057837000000004, 2.0000000000000000}),
        Coordinate({34.000000000000000, 0.74057837999999998, 2.0000000000000000}),
        Coordinate({33.000000000000000, 0.91539934000000001, 2.2458822600000001}),
        Coordinate({33.054659219999998, 1.0000000000000000,  2.2324425200000002}),
        Coordinate({33.000000000000000, 0.0000000000000000,  2.2458822600000001}),
        Coordinate({34.000000000000000, 1.0000000000000000,  2.0000000000000000})
    };
    CoordinateIds path{ 3, 5, 0, 1, 2, 6, 4 };

    for (const auto& t : Slicer::buildTrianglesFromPath(cs, path)) {
        EXPECT_NE(0.0, utils::Geometry::area(utils::Geometry::asTriV(t, cs)));
    }

}

TEST_F(SlicerTest, buildTrianglesFromPath_2)
{
    Coordinates cs{
        Coordinate({34.000000000000000, 0.74057837000000004, 2.0000000000000000}),
        Coordinate({34.000000000000000, 0.74057837999999998, 2.0000000000000000}),
        Coordinate({34.141133750000002, 1.0000000000000000,  1.9652977199999999}),
        Coordinate({34.000000000000000, 1.0000000000000000,  2.0000000000000000})
    };
    CoordinateIds path{ 3, 1, 0, 2};

    for (const auto& t : Slicer::buildTrianglesFromPath(cs, path)) {
        EXPECT_NE(0.0, utils::Geometry::area(utils::Geometry::asTriV(t, cs)));
    }

}

TEST_F(SlicerTest, tri45_size1_grid) 
{
    Mesh in = buildTri45Mesh(1.0);
    

    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(1, countMeshElementsIf(out, isTriangle));
}

TEST_F(SlicerTest, tri45_2_size1_grid)
{
    Mesh m;
        m.grid = utils::GridTools::buildCartesianGrid(0.0, 3.0, 4);

    m.coordinates = {
        Coordinate({ 3.00, 0.00, 0.50 }),
        Coordinate({ 0.00, 3.00, 1.00 }),
        Coordinate({ 0.00, 3.00, 0.00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(5, countMeshElementsIf(out, isTriangle));
}

TEST_F(SlicerTest, tri45_3_size1_grid)
{
    Mesh m;
    m.grid = {
        utils::GridTools::linspace(-61.0, 61.0, 123),
        utils::GridTools::linspace(-59.0, 59.0, 119),
        utils::GridTools::linspace(-11.0, 11.0, 23)
    };
    m.coordinates = {
        Coordinate({ 14.000000000000000, -13.000000000000000, 1.0000000000000000 }),
        Coordinate({ 14.000000000000000, -13.000000000000000, 0.0000000000000000 }),
        Coordinate({ 11.000000000000000, -10.000000000000000, 0.50000000000000000 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(5, countMeshElementsIf(out, isTriangle));
}

TEST_F(SlicerTest, tri45_size05_grid) 
{
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ buildTri45Mesh(0.5) }.getMesh());

    EXPECT_EQ(3, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, tri45_size025_grid) 
{
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ buildTri45Mesh(0.25) }.getMesh());

    ASSERT_EQ(1, out.groups.size());
    EXPECT_EQ(countMeshElementsIf(out, isTriangle), out.groups[0].elements.size());
    EXPECT_EQ(10, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size1_grid)
{
    Mesh in = buildCubeSurfaceMesh(1.0);
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(12, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size05_grid)
{
    Mesh in = buildCubeSurfaceMesh(0.5);
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(48, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size3_grid)
{
    Mesh in = buildCubeSurfaceMesh(3.0);
    

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(12, countMeshElementsIf(out, isTriangle));
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, tri_less45_size025_grid)
{
    Mesh in = buildTri45Mesh(0.25);
    in.coordinates[0] = Coordinate({ 1.45000000e+00, 1.00000000e+00, 1.4500000e+00 });

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(11, countMeshElementsIf(out, isTriangle));
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, meshTrisOnGridBoundaries)
{
    Mesh m;
    m.grid = {
        std::vector<double>({0.0, 1.0, 2.0}),
        std::vector<double>({0.0, 1.0, 2.0}),
        std::vector<double>({0.0, 1.0, 2.0})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };

    {
        m.coordinates = {
            Coordinate({0.0, 0.0, 0.0}),
            Coordinate({1.0, 0.0, 0.0}),
            Coordinate({0.0, 1.0, 0.0})
        };

        Mesh sliced = Slicer{m}.getMesh();

        EXPECT_EQ(1, countMeshElementsIf(sliced, isTriangle));
    }

    {
        m.coordinates = {
            Coordinate({2.0, 2.0, 2.0}),
            Coordinate({2.0, 1.0, 2.0}),
            Coordinate({2.0, 2.0, 1.0})
        };

        Mesh sliced = Slicer{m}.getMesh();

        EXPECT_EQ(1, countMeshElementsIf(sliced, isTriangle));
    }
}

TEST_F(SlicerTest, tri_degenerate) 
{
    Mesh m;
    m.grid = {
        utils::GridTools::linspace(-61.0, 61.0, 123),
        utils::GridTools::linspace(-59.0, 59.0, 119),
        utils::GridTools::linspace(-11.0, 11.0, 23)
    };
    m.coordinates = {
        Coordinate({ +8.00000000e+00, -7.00000000e+00, +1.00000000e+00 }),
        Coordinate({ +5.25538121e+00, -2.43936816e+00, +1.00000000e+00 }),
        Coordinate({ +2.00000000e+00, -1.00000000e+00, +1.00000000e+00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({2, 1, 0}, Element::Type::Surface)
    };
 
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());
    
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cell_faces_are_crossed)
{
    Mesh m = buildProblematicTriMesh();
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}


TEST_F(SlicerTest, cell_faces_are_crossed_2)
{
    Mesh m = buildProblematicTriMesh2();
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cell_faces_are_crossed_3)
{
    Mesh m;
    m.grid = buildProblematicTriMesh2().grid;
    m.coordinates = {
        Coordinate({ +2.56922991e+01, -2.52166072e+01, -6.57364794e+00 }),
        Coordinate({ +2.71235688e+01, -2.90000000e+01, -6.95715551e+00 }),
        Coordinate({ +2.59161616e+01, -2.90000000e+01, -6.63363171e+00 })
    };
    m.groups = { Group{} };
    m.groups[0].elements = {
        Element{ {0, 1, 2} }
    };

    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ m }.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, canSliceLinesInAdjacentCellsFromTheSamePlane)
{

    // y                                      y
    // *-------------*-------------*          *-------------*-------------*
    // |             |    1        |          |             |    {1->2}   |
    // |             | _-‾         |          |             | _-‾         |
    // |            _┼‾            |  ->      |         {0.5->1}          |
    // |         _-‾ |             |          |         _-‾ |             |
    // |       0‾    |             |          |       0‾    |             |
    // |             |             |          |             |             |
    // *-------------*-------------* x        *-------------*-------------*  x  

    // z                                       z                                
    // *-------------*-------------*           *-------------*-------------*    
    // |             |             |           |             |             |    
    // |             |  2          |           |             | {2->3}      |    
    // |             |⟋            ⎸           ⎸             ⎸⟋            ⎸   
    // |            ⟋⎸             ⎸           |         {2.33->4}         |    
    // |          ⟋  ⎸             ⎸           |          ⟋  ⎸             ⎸    
    // |        ⟋    ⎸             ⎸           |        ⟋    ⎸             ⎸    
    // *------⟋------*-------------*  ->       *--{2.67->5}--*-------------*    
    // |    ⟋        ⎸             ⎸           ⎸    ⟋        ⎸             ⎸    
    // |   3         |             |           | {3->6}      |             |    
    // |             |             |           |             |             |   
    // |             |             |           |             |             |    
    // |             |             |           |             |             |    
    // |             |             |           |             |             |    
    // *-------------*-------------* x         *-------------*-------------* x        

    Mesh m;
    m.grid = {
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0})
    };
    m.coordinates = {
        Coordinate({ -2.4, -2.6, -5.0 }),   // 0 First Segment, First Point
        Coordinate({ +2.4, -1.2, -5.0 }),   // 1 First Segment, Final Point
        Coordinate({ +1.5, -5.0, +4.5 }),   // 2 Second Segment, First Point
        Coordinate({ -4.5, -5.0, -1.5 }),   // 3 Second Segment, Final Point
    };
    m.groups.resize(2);
    m.groups[0].elements = {
        Element{ {0, 1}, Element::Type::Line }
    };
    m.groups[1].elements = {
        Element{ {2, 3}, Element::Type::Line }
    };
    GridTools tools(m.grid);

    Coordinate distanceFirstSegment = m.coordinates[1] - m.coordinates[0];
    CoordinateDir xDistance = 0.0 - m.coordinates[0][0];
    CoordinateDir yDistance = xDistance * distanceFirstSegment[1] / distanceFirstSegment[0];
    Coordinate intersectionPointFirstSegment = Coordinate({0.0, m.coordinates[0][1] + yDistance, -5.0});

    Coordinate distanceSecondSegment = m.coordinates[3] - m.coordinates[2];
    CoordinateDir secondPointXComponent = 0.0 - m.coordinates[2][0];
    CoordinateDir secondPointZComponent = secondPointXComponent * distanceSecondSegment[2] / distanceSecondSegment[0];
    CoordinateDir thirdPointZComponent = 0.0 - m.coordinates[3][2];
    CoordinateDir thirdPointXComponent = thirdPointZComponent * distanceSecondSegment[0] / distanceSecondSegment[2];
    Coordinate firstIntersectionPointSecondSegment = Coordinate({ 0.0, -5.0, m.coordinates[2][2] + secondPointZComponent });
    Coordinate secondIntersectionPointSecondSegment = Coordinate({ m.coordinates[3][2] - thirdPointXComponent, -5.0, 0.0 });



    Coordinates expectedCoordinates = {
        m.coordinates[0],                       // 0 First Segment, First Point
        intersectionPointFirstSegment,          // 1 First Segment, Intersection Point
        m.coordinates[1],                       // 2 First Segment, Final Point
        m.coordinates[2],                       // 3 Second Segment, First Point
        firstIntersectionPointSecondSegment,    // 4 Second Segment, Intersection First Point
        secondIntersectionPointSecondSegment,   // 5 Second Segment, Intersection Second Point
        m.coordinates[3],                       // 6 Second Segment, Final Point
    };

    Relatives expectedRelatives = tools.absoluteToRelative(expectedCoordinates);

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
        },
        {
            Element({3, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
            Element({5, 6}, Element::Type::Line),
        },
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = Slicer{ m }.getMesh());

    EXPECT_FALSE(containsDegenerateTriangles(resultMesh));

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    ASSERT_EQ(resultMesh.groups[0].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 3);

    for (std::size_t i = 0; i < expectedRelatives.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis]);
        }
    }

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        EXPECT_TRUE(resultGroup.elements[1].isLine());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}

TEST_F(SlicerTest, canSliceLinesInAdjacentCellThatPassThoroughPoints)
{

    // y                                          y
    // *---------------*---------------*          *---------------*---------------*
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |  ->      |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // |               |               |          |               |               |
    // *---0======================1----* x        *---0======={0.5->1}===={1->2}--* x
    // 
    // z                                 z                              
    // *-------------*-------------*     *-------------*-------------*  
    // |             |             |     |             |             |  
    // |             |             |     |             |             |  
    // |             |       2     |     |             |    {2->3}   |  
    // |             |     ⟋       ⎸     ⎸             ⎸     ⟋       ⎸  
    // |             |   ⟋         ⎸     ⎸             ⎸   ⟋         ⎸  
    // |             | ⟋           ⎸     ⎸             ⎸ ⟋           ⎸  
    // *-------------⟋-------------*  -> *--------- {2.5->4}---------*  
    // |           ⟋ ⎸             ⎸     ⎸           ⟋ ⎸             ⎸  
    // |         ⟋   ⎸             ⎸     ⎸         ⟋   ⎸             ⎸  
    // |       ⟋     ⎸             ⎸     ⎸       ⟋     ⎸             ⎸  
    // |      3      |             |     |    {3->5}   |             |  
    // |             |             |     |             |             |  
    // |             |             |     |             |             |  
    // *-------------*-------------* x   *-------------*-------------* x

    Mesh m;
    m.grid = {
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0})
    };
    m.coordinates = {
        Coordinate({ -4.5, -5.0, -5.0 }),   // 0 First Segment, First Point
        Coordinate({ +4.5, -5.0, -5.0 }),   // 1 First Segment, Final Point
        Coordinate({ +3.0, -5.0, +3.0 }),   // 2 Second Segment, First Point
        Coordinate({ -3.0, -5.0, -3.0 }),   // 3 Second Segment, Final Point
    };
    m.groups.resize(2);
    m.groups[0].elements = {
        Element{ {0, 1}, Element::Type::Line }
    };
    m.groups[1].elements = {
        Element{ {2, 3}, Element::Type::Line }
    };
    GridTools tools(m.grid);

    Coordinate intersectionPointFirstSegment = Coordinate({ 0.0, -5.0, -5.0 });
    Coordinate intersectionPointSecondSegment = Coordinate({ 0.0, -5.0, 0.0 });



    Coordinates expectedCoordinates = {
        m.coordinates[0],                       // 0 First Segment, First Point
        intersectionPointFirstSegment,          // 1 First Segment, Intersection Point
        m.coordinates[1],                       // 2 First Segment, Final Point
        m.coordinates[2],                       // 3 Second Segment, First Point
        intersectionPointSecondSegment,         // 4 Second Segment, Intersection Point
        m.coordinates[3],                        // 5 Second Segment, Final Point
    };

    Relatives expectedRelatives = tools.absoluteToRelative(expectedCoordinates);

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
        },
        {
            Element({3, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
        },
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = Slicer{ m }.getMesh());

    EXPECT_FALSE(containsDegenerateTriangles(resultMesh));

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    ASSERT_EQ(resultMesh.groups[0].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 2);

    for (std::size_t i = 0; i < expectedRelatives.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis]);
        }
    }

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        EXPECT_TRUE(resultGroup.elements[1].isLine());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}



TEST_F(SlicerTest, canSliceLinesInAdjacentCellsWithThreeDimensionalMovement)
{
 
    //              *-------------*-------------*                   *-------------*-------------*
    //             /|            /|            /|                  /|            /|            /|
    //            / |           / |           / |                 / |           / |           / |
    //           /  |          /  |          /  |                /  |          /  |          /  |
    //          *---┼---------*---┼---------*   |               *---┼---------*---┼---------*   |
    //         /|   |        /|   |      1 /|   |              /|   |        /|   |      4 /|   |
    //      Z / |   *-------/-┼---*----/-┼/-┼---*           Z / |   *-------/-┼---*----/-┼/-┼---*
    //       /  |  /|      /  |  /|  /   /  |  /|            /  |  /|      /  |  /|  +{3}/  |  /|
    //    +5*---┼---┼-----*---┼---┼/----*¦  | / |         +5*---┼---┼-----*---┼---┼/-┼--*¦  | / |
    //      |   |/  |     |   |/ /|     |¦  |/  |           |   |/  |     |   |/ /|  ¦  |¦  |/  |
    //      |   *---┼-----┼---*/--┼-----┼┼--*   |           |   *---┼-----┼---*/--┼--┼--┼┼--*   |
    //      |  /| +5|Y    |  ⫽|   ⎸     ⎸¦ /⎸   ⎸    ->     ⎸ ┈┈┼┈┈┈┼Y┈┈┈┈┼┈┈+{2} ⎸  ¦  ⎸¦ /⎸   ⎸ 
    //      | / |   *-----┼//-┼---*-----┼┼/-┼---*           | / | +5*-----┼//-┼---*--┼--┼┼/-┼---*
    //      |/  |  /     /|/  |  /      |¦  |  /            |/  |  /     +{1} |  /   ¦  |¦  |  /
    //      *---┼------/--*---┼---------*   | /             *---┼------/-┼*---┼------¦--*   | /
    //      |   |/   0    |   |/        |   |/              |   |/   0   ¦|   |/        |   |/
    //      |   *----┼----┼---*---------┼---*               |   *----┼---┼┼---*---------┼---*
    //      |  /     ¦    |  /          |  /                |  /     ¦    |  /          |  /
    //      | /           | /           | /                 | /           | /           | /
    //      |/            |/            |/                  |/            |/            |/
    //    -5*-------------*-------------* +5  X           -5*-------------*-------------* +5  X

    Mesh m;
    m.grid = {
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0})
    };
    m.coordinates = {
        Coordinate({ -2.75, -2.75, -1.50 }),   // 0 First Segment, First Point
        Coordinate({ +2.40, +1.50, +1.50 }),   // 1 First Segment, Final Point
    };

    m.groups.resize(1);
    m.groups[0].elements = {
        Element{ {0, 1}, Element::Type::Line }
    };
    GridTools tools(m.grid);

    Elements expectedElements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2, 3}, Element::Type::Line),
        Element({3, 4}, Element::Type::Line),
    };

    Mesh resultMesh;

    Relatives relativeCoordinates = tools.absoluteToRelative(m.coordinates);

    ASSERT_NO_THROW(resultMesh = Slicer{ m }.getMesh());

    EXPECT_FALSE(containsDegenerateTriangles(resultMesh));

    ASSERT_EQ(resultMesh.coordinates.size(), 5);
    ASSERT_EQ(resultMesh.groups.size(), 1);

    ASSERT_EQ(resultMesh.groups[0].elements.size(), 4);

    ASSERT_EQ(resultMesh.coordinates[0][0], relativeCoordinates[0][0]);
    ASSERT_EQ(resultMesh.coordinates[0][1], relativeCoordinates[0][1]);
    ASSERT_EQ(resultMesh.coordinates[0][2], relativeCoordinates[0][2]);

    ASSERT_LT(resultMesh.coordinates[1][0], 1.0);
    ASSERT_LT(resultMesh.coordinates[1][1], 1.0);
    ASSERT_EQ(resultMesh.coordinates[1][2], 1.0);

    ASSERT_EQ(resultMesh.coordinates[2][0], 1.0);
    ASSERT_LT(resultMesh.coordinates[2][1], 1.0);
    ASSERT_GT(resultMesh.coordinates[2][2], 1.0);

    ASSERT_GT(resultMesh.coordinates[3][0], 1.0);
    ASSERT_EQ(resultMesh.coordinates[3][1], 1.0);
    ASSERT_GT(resultMesh.coordinates[3][2], 1.0);

    ASSERT_EQ(resultMesh.coordinates[4][0], relativeCoordinates[1][0]);
    ASSERT_EQ(resultMesh.coordinates[4][1], relativeCoordinates[1][1]);
    ASSERT_EQ(resultMesh.coordinates[4][2], relativeCoordinates[1][2]);

    auto& resultGroup = resultMesh.groups[0];


    for (std::size_t e = 0; e < expectedElements.size(); ++e) {
        auto& resultElement = resultGroup.elements[e];
        auto& expectedElement = expectedElements[e];

        EXPECT_TRUE(expectedElement.isLine());

        for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
            EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
        }
    }
}

TEST_F(SlicerTest, preserves_topological_closedness_for_alhambra)
{
    auto m = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
  
    m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    auto slicedMesh = Slicer{m}.getMesh();

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
}

TEST_F(SlicerTest, preserves_topological_closedness_for_sphere)
{
    auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));

    auto slicedMesh = Slicer{m}.getMesh();

    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
}

}