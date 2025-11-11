#include "MeshFixtures.h"
#include "gtest/gtest.h"

#include "Slicer.h"
#include "Collapser.h"
#include "Geometry.h"
#include "MeshTools.h"
#include "app/vtkIO.h"
#include "utils/RedundancyCleaner.h"
#include "utils/CoordGraph.h"

namespace meshlib::core {

using namespace meshFixtures;
using namespace utils;
using namespace meshTools;

class SlicerTest : public ::testing::Test {
public:
protected:

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

    static std::size_t countContours(const Mesh& m)
    {
        std::size_t res = 0;
        for (const auto& g : m.groups) {
            res += CoordGraph{g.elements}.getBoundaryGraph().split().size();
        }
        return res;
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
    Mesh m = buildTri45Mesh(1.0);
    

    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(1, countMeshElementsIf(out, isTriangle));
    EXPECT_EQ(countContours(m), countContours(out));
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
    EXPECT_EQ(countContours(m), countContours(out));
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
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, tri45_size05_grid) 
{
    Mesh m = buildTri45Mesh(0.5);
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ m }.getMesh());

    EXPECT_EQ(3, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, tri45_size025_grid) 
{
    auto m = buildTri45Mesh(0.25);
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ m }.getMesh());

    ASSERT_EQ(1, out.groups.size());
    EXPECT_EQ(countMeshElementsIf(out, isTriangle), out.groups[0].elements.size());
    EXPECT_EQ(10, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, cube1x1x1_size1_grid)
{
    Mesh m = buildCubeSurfaceMesh(1.0);
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(12, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, cube1x1x1_size05_grid)
{
    Mesh m = buildCubeSurfaceMesh(0.5);
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(48, countMeshElementsIf(out, isTriangle));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, cube1x1x1_size3_grid)
{
    Mesh m = buildCubeSurfaceMesh(3.0);
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(12, countMeshElementsIf(out, isTriangle));
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
}

TEST_F(SlicerTest, tri_less45_size025_grid)
{
    Mesh m = buildTri45Mesh(0.25);
    m.coordinates[0] = Coordinate({ 1.45000000e+00, 1.00000000e+00, 1.4500000e+00 });

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(11, countMeshElementsIf(out, isTriangle));
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
    EXPECT_EQ(countContours(m), countContours(out));
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
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis])
                << "Current coordinate: #" << i << std::endl
                << "Current Axis: #" << axis << std::endl;
        }
    }

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            EXPECT_TRUE(resultElement.isLine())
                << "Current Group: #" << g << std::endl
                << "Current Element: #" << e << std::endl;

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                    << "Current Group: #" << g << std::endl
                    << "Current Element: #" << e << std::endl
                    << "Current Vertex: #" << v << std::endl;
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
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis])
                << "Current coordinate: #" << i << std::endl
                << "Current Axis: #" << axis << std::endl;
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
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                    << "Current Group: #" << g << std::endl
                    << "Current Element: #" << e << std::endl
                    << "Current Vertex: #" << v << std::endl;
            }
        }
    }
}



TEST_F(SlicerTest, keepsNodesIntact)
{

    // z
    // *-------------*-------------*
    // |             |             |
    // |             |             |
    // |             |       2     |
    // |             |             |
    // |             |             |
    // |             |             |
    // *-------------*-------------*
    // |             |             |
    // |             |             |
    // |             |             |
    // |      3      |             |
    // |             |             |  
    // |             |             |  
    // *---0---------*--------1----* x

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
        Element{ {0}, Element::Type::Node },
        Element{ {1}, Element::Type::Node }
    };
    m.groups[1].elements = {
        Element{ {2}, Element::Type::Node },
        Element{ {3}, Element::Type::Node }
    };
    GridTools tools(m.grid);

    Coordinate intersectionPointFirstSegment = Coordinate({ 0.0, -5.0, -5.0 });
    Coordinate intersectionPointSecondSegment = Coordinate({ 0.0, -5.0, 0.0 });

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = Slicer{ m }.getMesh());

    EXPECT_FALSE(containsDegenerateTriangles(resultMesh));

    ASSERT_EQ(resultMesh.coordinates.size(), m.coordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), m.groups.size());


    Relatives expectedRelatives = tools.absoluteToRelative(m.coordinates);

    for (std::size_t i = 0; i < m.coordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis])
                << "Current coordinate: #" << i << std::endl
                << "Current Axis: #" << axis << std::endl;
        }
    }

    for (std::size_t g = 0; g < m.groups.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = m.groups[g];
        ASSERT_EQ(resultGroup.elements.size(), expectedGroup.elements.size());

        for (std::size_t e = 0; e < expectedGroup.elements.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup.elements[e];

            ASSERT_TRUE(resultElement.isNode());
            ASSERT_EQ(resultElement.vertices[0], expectedElement.vertices[0]);
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
    //      |  /| +5|Y    |  ⫽⎸   ⎸     ⎸¦ /⎸   ⎸    ->     ⎸ ┈┈┼┈┈┈┼Y┈┈┈┈┼┈┈+{2} ⎸  ¦  ⎸¦ /⎸   ⎸ 
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

        EXPECT_TRUE(expectedElement.isLine()) << "Current Element: #" << e << std::endl;

        for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
            EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                << "Current Element: #" << e << std::endl
                << "Current Vertex: #" << v << std::endl;
        }
    }
}



TEST_F(SlicerTest, canKeepOppositeLinesInLineDimensionPolicy)
{

    // z                                       z                                   
    // *-------------*-------------*          *-------------*----------------*    
    // |             ⎹             |          |              |               ⎸  
    // |             ⎹  1          |          |              | {1->3}        ⎸  
    // |             ⎹⫽            ⎸          ⎸              ⎸⫽             ⎸    
    // |        ⩘   ⫽⎸   3        ⎹          ⎹          {0.66->2} {3->6}    ⎸   
    // |       /   ⫽/|  ⫽⩘        |          |           ⫽  ⎸   ⫽         ⎹     
    // |         ⫽ ⩗ ⎸⫽ /         |          |         ⫽    ⎸ ⫽           ⎹     
    // *-------⫽-----*-------------*  ->      *-{0.33->1}--{2.5->5}---------*    
    // |      ⫽    ⫽⎹             ⎹          ⎹     ⫽      ⫽ ⎸              |    
    // |     0    ⫽  ⎸             ⎸          ⎸    0      ⫽  |              |    
    // |         2   |             |          |        {2->4} |              |   
    // |             |             |          |               |              |    
    // |             |             |          |               |              |    
    // |          <- |             |          |               |              |    
    // *--------4====*====5--------* x        *------{4->7}===*===={5->8}----* x    
    //            ->
    // 
    // 
    //              *-------------*--------------*                   *-------------*-------------*
    //             /|            /|             /⎸                  /|            /|            /|
    //            / |           / |            / ⎸                 / |           / |           / |
    //           /  |          /  |           /  ⎸                /  |          /  |          /  |
    //          *---┼---------*---┼----------*   ⎸               *---┼---------*---┼---------*   |
    //         /|   |        /|   |       7 /⎸   ⎸              /|   |        /|   |      10/|   |
    //      Z / |   *-------/-┼---*----⫽-┼/-┼---*           Z / |   *-------/-┼---*----⫽-┼/-┼---*
    //       /  |  /|      /  |  /|  ⫽   /  |  /|            /  |  /|      /  |  /|  +{9}/  ⎹  /⎹
    //    +5*---┼---┼-----*---┼---┼⫽----*¦  | / |         +5*---┼---┼-----*---┼- -┼⫽-┼--*¦  | / |
    //      |   |/  |     |   |/ ⫽|     |¦  |/  |           |   |/  |     |   |/⫽⎹   ¦  |¦  |/  |
    //      |   *---┼-----┼---*⫽--┼-----┼┼--*   |           |   *---┼-----┼---*⫽-┼---┼--┼┼--*   |
    //      |  /| +5|Y    |  ⫻⎸   ⎸     ⎸¦ /⎸   ⎸    ->     ⎸ ┈┈┼┈┈┈┼Y┈┈┈┈┼┈┈+{8}⎹   ¦  ⎹⎹  /⎸  ⎹  
    //      | / |   *-----┼⫽/-┼---*-----┼┼/-┼---*           | / | +5*-----┼⫽/-┼--*---┼--┼┼/-┼---*
    //      |/  |  /     ⫽⎸/  |  /      |¦  |  /            |/  |  /     +{7} |  /   ¦  ⎹¦  ⎹  /
    //      *---┼------⫽--*---┼---------*   | /             *---┼------⫽-┼*--┼------¦---*   | /
    //      |   |/    6   |   ⎹/        |   ⎹/              |   |/   6   ¦|   |/         |   |/
    //      |   *-----┼---┼---*---------┼---*               |   *----┼---┼┼---*----------┼---*
    //      |  /      ¦   |  /          |  /                |  /     ¦    |  /           |  /
    //      | /           | /           | /                 | /           | /            | /
    //      |/            |/            |/                  |/            |/             |/
    //    -5*-------------*-------------* +5  X           -5*-------------*--------------* +5  X

    Mesh m;
    m.grid = {
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0}),
        std::vector<double>({-5.0, 0.0, 5.0})
    };
    m.coordinates = {
        Coordinate({ -4.5,  -5.0,  -1.5  }),    // First Line, Vertex 0, Second Line, Vertex 0
        Coordinate({ +1.5,  -5.0,  +4.5  }),    // First Line, Vertex 1, Second Line, Vertex 1
        Coordinate({ -3.0,  -5.0,  -3.0  }),    // Third Line, Vertex 1, Fourth Line, Vertex 0
        Coordinate({ +3.0,  -5.0,  +3.0  }),    // Third Line, Vertex 0, Fourth Line, Vertex 1
        Coordinate({ -4.5,  -5.0,  -5.0  }),    // Fifth Line, Vertex 0, Sixth Line, Vertex 0
        Coordinate({ +4.5,  -5.0,  -5.0  }),    // Fifth Line, Vertex 1, Sixth Line, Vertex 1
        Coordinate({ -2.75, -2.75, -1.50 }),    // Seventh Line, Vertex 0, Eighth Line, Vertex 0
        Coordinate({ +2.40, +1.50, +1.50 }),    // Seventh Line, Vertex 1, Eighth Line, Vertex 1
    };

    m.groups.resize(2);
    m.groups[0].elements = {
        Element{ {0, 1}, Element::Type::Line }, // First Line
        Element{ {1, 0}, Element::Type::Line }, // Second Line
        Element{ {2, 3}, Element::Type::Line }, // Third Line
        Element{ {3, 2}, Element::Type::Line }, // Fourth Line
        Element{ {4, 5}, Element::Type::Line }, // Fifth Line
        Element{ {5, 4}, Element::Type::Line }, // Sixth Line
        Element{ {6, 7}, Element::Type::Line }, // Seventh Line
        Element{ {7, 6}, Element::Type::Line }, // Eighth Line
    };
    m.groups[1].elements = {
        Element{ {0, 1}, Element::Type::Line }, // First Line
        Element{ {1, 0}, Element::Type::Line }, // Second Line
        Element{ {2, 3}, Element::Type::Line }, // Third Line
        Element{ {3, 2}, Element::Type::Line }, // Fourth Line
        Element{ {4, 5}, Element::Type::Line }, // Fifth Line
        Element{ {5, 4}, Element::Type::Line }, // Sixth Line
        Element{ {6, 7}, Element::Type::Line }, // Seventh Line
        Element{ {7, 6}, Element::Type::Line }, // Eighth Line
    };
    GridTools tools(m.grid);

    Coordinate distanceFirstLine = m.coordinates[1] - m.coordinates[0];
    CoordinateDir firstPointXComponent = 0.0 - m.coordinates[0][0];
    CoordinateDir firstPointZComponent = firstPointXComponent * distanceFirstLine[2] / distanceFirstLine[0];
    CoordinateDir secondPointZComponent = 0.0 - m.coordinates[1][2];
    CoordinateDir secondPointXComponent = secondPointZComponent * distanceFirstLine[0] / distanceFirstLine[2];
    Coordinate firstIntersectionPointFirstLine = Coordinate({ secondPointXComponent + m.coordinates[1][0], -5.0, 0.0 });
    Coordinate secondIntersectionPointFirstLine = Coordinate({ 0.0, -5.0, firstPointZComponent + m.coordinates[0][2] });

    Coordinates expectedCoordinates = {
        m.coordinates[0],                       // 0 First Line, First Point
        firstIntersectionPointFirstLine,        // 1 First Line, First Intersection Point
        secondIntersectionPointFirstLine,       // 2 First Line, Second Intersection Point
        m.coordinates[1],                       // 3 First Line, Final Point
        m.coordinates[2],                       // 4 Third Line, First Point
        Coordinate({ 0, -5.0, 0 }),             // 5 Third Line, Intersection Point
        m.coordinates[3],                       // 6 Third Line, Final Point
        m.coordinates[4],                       // 7 Fifth Line, Final Point
        Coordinate({ 0, -5.0, -5.0 }),          // 8 Fifth Line, Intersection Point
        m.coordinates[5],                       // 9 Fifth Line, Final Point
        m.coordinates[6],                       // 10 Seventh Line, First Point
        Coordinate({0, 0, 0}),                  // 11 Seventh Line, First Intersection Point
        Coordinate({0, 0, 0}),                  // 12 Seventh Line, Second Intersection Point
        Coordinate({0, 0, 0}),                  // 13 Seventh Line, Third Intersection Point
        m.coordinates[7],                       // 14 Seventh Line, Final Point
    };

    Relatives expectedRelatives = tools.absoluteToRelative(expectedCoordinates);

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),

            Element({3, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),

            Element({4, 5}, Element::Type::Line),
            Element({5, 6}, Element::Type::Line),

            Element({6, 5}, Element::Type::Line),
            Element({5, 4}, Element::Type::Line),

            Element({7, 8}, Element::Type::Line),
            Element({8, 9}, Element::Type::Line),

            Element({9, 8}, Element::Type::Line),
            Element({8, 7}, Element::Type::Line),

            Element({10, 11}, Element::Type::Line),
            Element({11, 12}, Element::Type::Line),
            Element({12, 13}, Element::Type::Line),
            Element({13, 14}, Element::Type::Line),

            Element({14, 13}, Element::Type::Line),
            Element({13, 12}, Element::Type::Line),
            Element({12, 11}, Element::Type::Line),
            Element({11, 10}, Element::Type::Line),
        },
        {

            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),

            Element({4, 5}, Element::Type::Line),
            Element({5, 6}, Element::Type::Line),

            Element({7, 8}, Element::Type::Line),
            Element({8, 9}, Element::Type::Line),

            Element({10, 11}, Element::Type::Line),
            Element({11, 12}, Element::Type::Line),
            Element({12, 13}, Element::Type::Line),
            Element({13, 14}, Element::Type::Line),
        }
    };

    Mesh resultMesh;

    Relatives relativeCoordinates = tools.absoluteToRelative(m.coordinates);

    std::vector<Element::Type> dimensions({ Element::Type::Line, Element::Type::Surface });

    ASSERT_NO_THROW(resultMesh = Slicer(m, dimensions).getMesh());

    EXPECT_FALSE(containsDegenerateTriangles(resultMesh));
    RedundancyCleaner::cleanCoords(resultMesh);
    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());



    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    for (std::size_t i = 0; i < (m.coordinates.size() - 4); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_DOUBLE_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis])
                << "Current coordinate: #" << i << std::endl
                << "Current Axis: #" << axis << std::endl;
        }
    }

    EXPECT_LT(resultMesh.coordinates[11][X], 1.0);
    EXPECT_LT(resultMesh.coordinates[11][Y], 1.0);
    EXPECT_EQ(resultMesh.coordinates[11][Z], 1.0);

    EXPECT_EQ(resultMesh.coordinates[12][X], 1.0);
    EXPECT_LT(resultMesh.coordinates[12][Y], 1.0);
    EXPECT_GT(resultMesh.coordinates[12][Z], 1.0);

    EXPECT_GT(resultMesh.coordinates[13][X], 1.0);
    EXPECT_EQ(resultMesh.coordinates[13][Y], 1.0);
    EXPECT_GT(resultMesh.coordinates[13][Z], 1.0);

    EXPECT_DOUBLE_EQ(resultMesh.coordinates[14][X], expectedRelatives[14][X]);
    EXPECT_DOUBLE_EQ(resultMesh.coordinates[14][Y], expectedRelatives[14][Y]);
    EXPECT_DOUBLE_EQ(resultMesh.coordinates[14][Z], expectedRelatives[14][Z]);

    
    
    for (GroupId g = 0; g < resultMesh.groups.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        ASSERT_EQ(resultGroup.elements.size(), expectedGroup.size()) << "Current Group: #" << g << std::endl;

        for (ElementId e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            EXPECT_TRUE(expectedElement.isLine())
                << "Current Group: #" << g << std::endl
                << "Current Element: #" << e << std::endl;

            for (CoordinateId v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                    << "Current Group: #" << g << std::endl
                    << "Current Element: #" << e << std::endl
                    << "Current Vertex: #" << v << std::endl;
            }
        }
    }
}


TEST_F(SlicerTest, preserves_topological_closedness_for_alhambra)
{
    auto m = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
    
    m.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    m.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    m.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    auto slicedMesh = Slicer{m}.getMesh();
    
    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
}

TEST_F(SlicerTest, preserves_topological_closedness_for_sphere)
{
    auto m = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    auto slicedMesh = Slicer{m}.getMesh();
    
    EXPECT_TRUE(meshTools::isAClosedTopology(m.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}

TEST_F(SlicerTest, sphere_case_patch_contour_check_1)
{
    Mesh m;
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }
    m.coordinates = {
        Coordinate({ 16.1734, -45.5077, 12.9410 }),
        Coordinate({ 27.8515, -39.4566, 12.9410 }),
        Coordinate({ 12.9214, -38.4466, 25.0000 }),
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };

    auto slicedMesh = Slicer{m}.getMesh();
    
    EXPECT_EQ(countContours(m), countContours(slicedMesh));

    //For debugging.
	// meshTools::convertToAbsoluteCoordinates(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", slicedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}

TEST_F(SlicerTest, sphere_case_patch_contour_check_2)
{
    Mesh m;
    for (auto x: {X,Y,Z}) {
        m.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }
    m.coordinates = {
        Coordinate({ -2.95498, 43.20030, 25.0000}),
        Coordinate({ -7.19326, 34.61580, 35.3553}),
        Coordinate({  2.41273, 35.27290, 35.3553}),
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };

    auto slicedMesh = Slicer{m}.getMesh();
    
    EXPECT_EQ(countContours(m), countContours(slicedMesh));

    // For debugging.
	meshTools::convertToAbsoluteCoordinates(slicedMesh);
	vtkIO::exportMeshToVTU("sliced.vtk", slicedMesh);

	auto contourMesh = meshTools::buildMeshFromContours(slicedMesh);
	vtkIO::exportMeshToVTU("contour.vtk", contourMesh);
}

}