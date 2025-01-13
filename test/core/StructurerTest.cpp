#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

#include "MeshFixtures.h"

#include "Structurer.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace core;
using namespace utils;
using namespace meshFixtures;

namespace meshlib::core {

class StructurerTest : public ::testing::Test {
protected:
};

TEST_F(StructurerTest, calculateStructuredCoordinateInExactSteps)
{
    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 6;
    float step = 2.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    Structurer structurer{ mesh };
    ASSERT_NO_THROW(structurer.grid());

    for (CellDir i = 0; i < mesh.grid[X].size(); ++i) {
        CoordinateDir xDirection = RelativeDir(i);

        for (CellDir j = 0; j < mesh.grid[Y].size(); ++j) {
            CoordinateDir yDirection = RelativeDir(j);

            for (CellDir k = 0; k < mesh.grid[Z].size(); ++k) {
                CoordinateDir zDirection = RelativeDir(k);

                Relative coordinate({ xDirection, yDirection, zDirection });
                Cell expectedCell({ i, j, k });

                auto resultCell = structurer.calculateStructuredCell(coordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedCell[axis]);
                }
            }
        }
    }
}

TEST_F(StructurerTest, calculateStructuredCoordinateBetweenSteps)
{
    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);
    float relativeStep = 1.0;
    float quarterStep = relativeStep / 4.0f;
    float halfStep = relativeStep / 2.0f;
    float threeQuarterStep = relativeStep * 3.0f / 4.0f;

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    Structurer structurer{ mesh };
    ASSERT_NO_THROW(structurer.grid());

    for (CellDir i = 0; i < mesh.grid[X].size() - 1; ++i) {
        for (CellDir j = 0; j < mesh.grid[Y].size() - 1; ++j) {
            for (CellDir k = 0; k < mesh.grid[Z].size() - 1; ++k) {
                Cell expectedLowerCell({ i, j, k });
                Cell expectedUpperCell({ i + 1, j + 1, k + 1 });

                Coordinate coordinateBase;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    coordinateBase[axis] = RelativeDir(expectedLowerCell[axis]);
                }

                Coordinate newCoordinate;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    newCoordinate[axis] = coordinateBase[axis] + quarterStep;
                }

                auto resultCell = structurer.calculateStructuredCell(newCoordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedLowerCell[axis]);
                }

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    newCoordinate[axis] = coordinateBase[axis] + halfStep;
                }

                resultCell = structurer.calculateStructuredCell(newCoordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedUpperCell[axis]);
                }

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    newCoordinate[axis] = coordinateBase[axis] + threeQuarterStep;
                }

                resultCell = structurer.calculateStructuredCell(newCoordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedUpperCell[axis]);
                }
            }
        }
    }
}

TEST_F(StructurerTest, transformSingleSegmentsIntoSingleStructuredElements)
{

    // *---------*      2---------*
    // |  2      |      ║         |
	// |  |      |      ║         |
    // |  |  _-1 |  ->  ║         |
	// |  0-‾    |      ║         |
    // *---------*      0=========1

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.3, 0.3, 0.0 }),
        Coordinate({ 0.7, 0.4, 0.0 }),
        Coordinate({ 0.3, 0.7, 0.0 }),
        Coordinate({ 0.3, 0.0, 0.7 }),
    };
    mesh.groups = { Group(), Group(), Group() };
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({0, 2}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({0, 3}, Element::Type::Line)
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 }),
    };

    Elements expectedElements = {
        Element({0, 1}, Element::Type::Line),
        Element({0, 2}, Element::Type::Line),
        Element({0, 3}, Element::Type::Line)
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 1);
    }                                 

    for (std::size_t g = 0; g < expectedCoordinates.size(); ++g) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[g][axis], expectedCoordinates[g][axis]);
        }
    }
    

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedElement = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
            EXPECT_EQ(resultGroup.elements[0].vertices[v], expectedElement.vertices[v]);
        }
    }
}

TEST_F(StructurerTest, transformSingleSegmentsIntoTwoStructuredElements)
{

    // *-----------*  {0.5->1}======{1->2}     *-----------*       *----------{3->2}
    // |      1    |      ║           |        |        3  |       |            ║
    // |     /     |      ║           |        |       /   |       |            ║
    // |    /      |  ->  ║           |        |      /    |   ->  |            ║
    // |  0        |      ║           |        |    2      |       |            ║
    // *-----------*      0-----------*        *-----------*    {2->0}======={2.5->3}

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1,  0.1,  0.0 }),    // 0 First and Third Segments, First Point, X-Y and X-Z Planes
        Coordinate({ 0.52, 0.9,  0.0 }),    // 1 First Segment, Final Point, X-Y Plane
        Coordinate({ 0.48, 0.1,  0.0 }),    // 2 Second and Fourth Segments, First Point, X-Y and X-Z Planes
        Coordinate({ 0.9,  0.9,  0.0 }),    // 3 Second Segment, Final Point, X-Y Plane
        Coordinate({ 0.52, 0.0,  0.9 }),    // 4 Third Segment, Final Point, X-Z Plane
        Coordinate({ 0.9,  0.0,  0.9 }),    // 5 Fourth Segment, Final Point, X-Z Plane
        Coordinate({ 0.0,  0.1,  0.1 }),    // 6 Fifth Segment, First Point, Y-Z Plane
        Coordinate({ 0.0,  0.52, 0.9 }),    // 7 Fifth Segment, First Point, Y-Z Plane
        Coordinate({ 0.0,  0.48, 0.1 }),    // 8 Sixth Segment, First Point, Y-Z Plane
        Coordinate({ 0.0,  0.9,  0.9 }),    // 9 Sixth Segment, First Point,  Y-Z Plane
    };
    mesh.groups.resize(6);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({0, 4}, Element::Type::Line)
    };
    mesh.groups[3].elements = {
        Element({2, 5}, Element::Type::Line)
    };
    mesh.groups[4].elements = {
        Element({6, 7}, Element::Type::Line)
    };
    mesh.groups[5].elements = {
        Element({8, 9}, Element::Type::Line)
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 All Segments, First Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 1 First and Sixth Segment, Middle Point, X-Y and Y-Z Planes
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First and Second Segment, Final Point, X-Y Plane
        Coordinate({ 1.0, 0.0, 0.0 }),    // 3 Second and Fourth Segment, Middle Point, X-Y and X-Z Planes
        Coordinate({ 0.0, 0.0, 1.0 }),    // 4 Third and Fifth Segment, Middle Point X-Z and Y-Z Planes
        Coordinate({ 1.0, 0.0, 1.0 }),    // 5 Third and Fourth Segments, Final Pint, X-Z Plane
        Coordinate({ 0.0, 1.0, 1.0 }),    // 6 Fifth and Sixth Segments, Final Point, Y-Z Plane
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
        },
        {
            Element({0, 3}, Element::Type::Line),
            Element({3, 2}, Element::Type::Line),
        },
        {
            Element({0, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
        },
        {
            Element({0, 3}, Element::Type::Line),
            Element({3, 5}, Element::Type::Line),
        },
        {
            Element({0, 4}, Element::Type::Line),
            Element({4, 6}, Element::Type::Line),
        },
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 6}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 2);
    }

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
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



TEST_F(StructurerTest, transformSingleSegmentsWithinDiagonalIntoTwoStructuredElements)
{

    // y                y                       y               y
    // *-------*        *--------{1->2}         *-------*    {0.5->1}====={1->2}
    // |      1|        |          ║            |      1|        ║           |
    // |    ╱  |        |          ║            |    ╱  |        ║           |
    // |  ╱    |   ->   |          ║            |  ╱    |   ->   ║           |
    // |0      |        |          ║            |0      |        ║           |
    // *-------* x      0======{0.5->1} x       *-------* z      0-----------* z

    // y                y                       y               y
    // *-------*        *----------0            *-------*    {0.5->1}========0
    // |      0|        |          ║            |      3|        ║           |
    // |    ╱  |        |          ║            |    ╱  |        ║           |
    // |  ╱    |   ->   |          ║            |  ╱    |   ->   ║           |
    // |1      |        |          ║            |4      |        ║           |
    // *-------* x    {1->2}==={0.5->1} x       *-------* z    {1->2}--------* z

    // y                y                      y                y               
    // *-------*     {1->2}===={0.5->1}         *-------*      {1->2}======{0.5->1} 
    // |1      |        |          ║            |1      ⎸         |            ║    
    // |  \    |        |          ║            |  \    ⎸         |            ║    
    // |    \  |   ->   |          ║            |    \  ⎸   ->    |            ║    
    // |      0|        |          ║            |      0⎸         |            ║    
    // *-------* x      *----------0 x          *-------* z       *------------0 z  

    // y                y                      y                y               
    // *-------*        0======{0.5->1}         *-------*         0======={0.5->1} 
    // |0      |        |          ║            |0      |         ⎹           ║    
    // |  \    |        |          ║            |  \    |         ⎹           ║    
    // |    \  |   ->   |          ║            |    \  |   ->    ⎹           ║    
    // |      1|        |          ║            |      1|         ⎹           ║    
    // *-------* x      *-------{1->2} x        *-------* z       *---------{1->2} z  

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1, 0.1, 0.0 }),    //  0 First Segment, First Point, Fourth Segment, Final Point, X-Y Plane
        Coordinate({ 0.9, 0.9, 0.0 }),    //  1 First Segment, Final Point, Fourth Segment, First Point, X-Y Plane
        Coordinate({ 0.1, 0.0, 0.1 }),    //  2 Second Segment, First Point, Fifth Segment, Final Point, X-Z Plane
        Coordinate({ 0.9, 0.0, 0.9 }),    //  3 Second Segment, Final Point, Fifth Segment, First Point,  X-Z Plane
        Coordinate({ 0.0, 0.1, 0.1 }),    //  4 Third Segment, First Point, Sixth Segment, Final Point, Y-Z Plane
        Coordinate({ 0.0, 0.9, 0.9 }),    //  5 Third Segment, Final Point, Sixth Segment, First Point, Y-Z Plane
        Coordinate({ 0.9, 0.1, 0.0 }),    //  6 Seventh Segment, First Point, Tenth Segment, Final Point, X-Y Plane
        Coordinate({ 0.1, 0.9, 0.0 }),    //  7 Seventh Segment, Final Point, Tenth Segment, First Point, X-Y Plane
        Coordinate({ 0.9, 0.0, 0.1 }),    //  8 Eighth Segment, First Point, Eleventh Segment, Final Point, X-Z Plane
        Coordinate({ 0.1, 0.0, 0.9 }),    //  9 Eighth Segment, Final Point, Eleventh Segment, First Point, X-Z Plane
        Coordinate({ 0.0, 0.1, 0.9 }),    // 10 Ninth Segment, First Point, Twelfth Segment, Final Point, Y-Z Plane
        Coordinate({ 0.0, 0.9, 0.1 }),    // 11 Ninth Segment, Final Point, Twelfth Segment, First Point, Y-Z Plane
    };
    mesh.groups.resize(12);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({4, 5}, Element::Type::Line)
    };
    mesh.groups[3].elements = {
        Element({1, 0}, Element::Type::Line)
    };
    mesh.groups[4].elements = {
        Element({3, 2}, Element::Type::Line)
    };
    mesh.groups[5].elements = {
        Element({5, 4}, Element::Type::Line)
    };
    mesh.groups[6].elements = {
        Element({6, 7}, Element::Type::Line)
    };
    mesh.groups[7].elements = {
        Element({8, 9}, Element::Type::Line)
    };
    mesh.groups[8].elements = {
        Element({10, 11}, Element::Type::Line)
    };
    mesh.groups[9].elements = {
        Element({7, 6}, Element::Type::Line)
    };
    mesh.groups[10].elements = {
        Element({9, 8}, Element::Type::Line)
    };
    mesh.groups[11].elements = {
        Element({11, 10}, Element::Type::Line)
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First, Second and Third Segments, First Point, Fourth, Fifth and Sixth Segments, Final Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First, Second, Fourth and Fifth Segments, Middle Point,
                                          //     Seventh and Eighth Segment, Start Point, Tenth and Eleventh Segments, Final Point, X-Y and X-Z Planes
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First Segment, Final Point, Fourth Segment, First Point
                                          //     Seventh and Tenth Segments, Middle Point, X-Y Plane
        Coordinate({ 1.0, 0.0, 1.0 }),    // 3 Second Segment, Final Point, Fifth Segment, First Point, 
                                          //     Eighth and Eleventh Segments, Middle Point, X-Z Plane
        Coordinate({ 0.0, 1.0, 0.0 }),    // 4 Third and Sixth Segments, Middle Point
                                          //     Seventh and Ninth Segment, Final Point, Tenth and Twelfth Segments, First Point, Y-Z Plane
        Coordinate({ 0.0, 1.0, 1.0 }),    // 5 Third Segment, Final Point, Sixth Segment, First Point, Ninth and Twelfth Segment, Middle Point Y-Z Plane
        Coordinate({ 0.0, 0.0, 1.0 }),    // 6 Eighth and Twelfth Segments, Final Point, Ninth and Eleventh Segments, First Point, X-Z Plane
        };                                //

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
        },
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 3}, Element::Type::Line),
        },
        {
            Element({0, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
        },
        {
            Element({2, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({3, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({5, 4}, Element::Type::Line),
            Element({4, 0}, Element::Type::Line),
        },
        {
            Element({1, 2}, Element::Type::Line),
            Element({2, 4}, Element::Type::Line),
        },
        {
            Element({1, 3}, Element::Type::Line),
            Element({3, 6}, Element::Type::Line),
        },
        {
            Element({6, 5}, Element::Type::Line),
            Element({5, 4}, Element::Type::Line),
        },
        {
            Element({4, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
        },
        {
            Element({6, 3}, Element::Type::Line),
            Element({3, 1}, Element::Type::Line),
        },
        {
            Element({4, 5}, Element::Type::Line),
            Element({5, 6}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 2);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];
            EXPECT_TRUE(resultElement.isLine());

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}


TEST_F(StructurerTest, transformSingleSegmentsIntoThreeStructuredElements)
{

    //     *-------------*               *----------{1->3}  
    //    /|            /|              /|            /║    
    //   / |           / |             / |           / ║    
    // z/  |        1 /  |           z/  |          /  ║    
    // *---┼-----==‾+*   |    ->     *---┼---------*   ║    
    // |   |y _-‾   ¦|   |           |   |y        |   ║    
    // |  _*==------┴┼---*           |{0.33->1}===={0.67->2}    
    // |0‾/          |  /            |  ⫽          ⎸  /
    // |¦/           | /             | ⫽           ⎸ /
    // |/            |/              |⫽            ⎸/
    // *-------------* x             0-------------* x      

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.0,  0.20, 0.10 }),    // 0 First, Third and Fourth Segments, First Point
        Coordinate({ 1.0,  0.90, 0.60 }),    // 1 First and Second Segments, Final Point
        Coordinate({ 0.0,  0.02, 0.48 }),    // 2 Second Segment, First Point
        Coordinate({ 0.0,  0.48, 0.02 }),    // 3 Third Segment, First Point
        Coordinate({ 0.52, 0.96, 0.8  }),    // 4 Third Segment, Final Point
        Coordinate({ 1.0,  0.52, 0.54 }),    // 5 Fourth Segment, Final Point
    };
    mesh.groups.resize(4);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
    };
    mesh.groups[1].elements = {
        Element({2, 1}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({3, 4}, Element::Type::Line)
    };
    mesh.groups[3].elements = {
        Element({0, 5}, Element::Type::Line)
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 All Segments Segments, First Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 1 First and Third Segments, Second Point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First Segment, Third Point
        Coordinate({ 1.0, 1.0, 1.0 }),    // 3 All Segments, Final Point
        Coordinate({ 0.0, 0.0, 1.0 }),    // 4 Second Segment, Second Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 5 Second and Fourth Segments, Third Point
        Coordinate({ 0.0, 1.0, 1.0 }),    // 6 Third Segment, Third Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 7 Fourth Segment, Second Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
        },
        {
            Element({0, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
            Element({5, 3}, Element::Type::Line),
        },
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 6}, Element::Type::Line),
            Element({6, 3}, Element::Type::Line),
        },
        {
            Element({0, 7}, Element::Type::Line),
            Element({7, 5}, Element::Type::Line),
            Element({5, 3}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 3);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        EXPECT_TRUE(resultGroup.elements[1].isLine());
        EXPECT_TRUE(resultGroup.elements[2].isLine());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e)
        {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}


TEST_F(StructurerTest, transformSingleSegmentsWithinDiagonalIntoThreeStructuredElements)
{

    //     *-------------*               *----------{1->3}  
    //    /|           1/|              /|            /║    
    //   / |          ╱¦ |             / |           / ║    
    // z/  |        ╱ /¦ |           z/  |          /  ║    
    // *---┼------╱--* ¦ |    ->     *---┼---------*   ║    
    // |   |y   ╱    | ¦ |           |   |y        |   ║    
    // |   *--╱------┼-┼-*           |   *---------{0.67->2}    
    // |  / ╱        |  /            |  /          |  ⫽     
    // | /╱          | /             | /           | ⫽      
    // |⌿0           ⎹/              ⎹/            ⎹⫽
    // *-------------* x             0========={0.33->1} x

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1, 0.1, 0.1 }),    // 0 First Segment, First Point and Eight Segment, Final Point
        Coordinate({ 0.9, 0.9, 0.9 }),    // 1 First Segment, Final Point and Eight Segment, First Point
        Coordinate({ 0.9, 0.1, 0.1 }),    // 2 Second Segment, First Point and Seventh Segment, Final point
        Coordinate({ 0.1, 0.9, 0.9 }),    // 3 Second Segment, Final Point and Seventh Segment, First Point
        Coordinate({ 0.1, 0.9, 0.1 }),    // 4 Third Segment, First Point and Sixth Segment, Final Point
        Coordinate({ 0.9, 0.1, 0.9 }),    // 5 Third Segment, Final Point and Sixth Segment, First Point
        Coordinate({ 0.1, 0.1, 0.9 }),    // 6 Fourth Segment, First Point and Fifth Segment, Final Point
        Coordinate({ 0.9, 0.9, 0.1 }),    // 7 Fourth Segment, Final Point and Fifth Segment, First Point
    };
    mesh.groups.resize(8);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line),
    };
    mesh.groups[2].elements = {
        Element({4, 5}, Element::Type::Line),
    };
    mesh.groups[3].elements = {
        Element({6, 7}, Element::Type::Line),
    };
    mesh.groups[4].elements = {
        Element({7, 6}, Element::Type::Line),
    };
    mesh.groups[5].elements = {
        Element({5, 4}, Element::Type::Line),
    };
    mesh.groups[6].elements = {
        Element({3, 2}, Element::Type::Line),
    };
    mesh.groups[7].elements = {
        Element({1, 0}, Element::Type::Line),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First Segment, First Point; and Eighth Segment, Final Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First Segment, Second Point; Second Segment, First Point, and Seventh Segment, Final point, Eighth Segment, Third Point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First, Sixth and Seventh Segments, Third Point; Second, Third and Eighth Segments, Second Point; Fourth Segment, Final Point; and Fifth Segment, First Point
        Coordinate({ 1.0, 1.0, 1.0 }),    // 3 First Segment, Final Point; Second, Third and Fourth Segments, Third Point; Fifth, Sixth and Seventh Segments, Second Point; and Eighth Segment, First Point
        Coordinate({ 0.0, 1.0, 1.0 }),    // 4 Second Segment, Final Point; Seventh Segment, First Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 5 Third Segment, First Point, and Sixth Segment, Final Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 6 Third Segment, Final Point; Fourth Segment, Second Point; Fifth Segment, Third Point, Sixth Segment, First Point
        Coordinate({ 0.0, 0.0, 1.0 }),    // 7 Fourth Segment, First Point and Fifth Segment, Final Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
        },
        {
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
            Element({3, 4}, Element::Type::Line),
        },
        {
            Element({5, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
            Element({3, 6}, Element::Type::Line),
        },
        {
            Element({7, 6}, Element::Type::Line),
            Element({6, 3}, Element::Type::Line),
            Element({3, 2}, Element::Type::Line),
        },
        {
            Element({2, 3}, Element::Type::Line),
            Element({3, 6}, Element::Type::Line),
            Element({6, 7}, Element::Type::Line),
        },
        {
            Element({6, 3}, Element::Type::Line),
            Element({3, 2}, Element::Type::Line),
            Element({2, 5}, Element::Type::Line),
        },
        {
            Element({4, 3}, Element::Type::Line),
            Element({3, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
        },
        {
            Element({3, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 3);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        EXPECT_TRUE(resultGroup.elements[1].isLine());
        EXPECT_TRUE(resultGroup.elements[2].isLine());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}


TEST_F(StructurerTest, transformSingleSegmentsParallelWithDiagonalIntoThreeStructuredElements)
{

    //     *----------1--*          {0.67->2}======={1->3}  
    //    /|         ╱¦ /|              /║            /|    
    //   / |       ╱  ¦╱ |             / ║           / |    
    // z/  |     ╱    ╱  |           z/  ║          /  |    
    // *---┼---╱-----*¦  |    ->     *---╫---------*   |    
    // |   |y╱       |¦  |           |   ║ y       |   |    
    // |   ╱---------┼┼--*           {0.67->2}-----┼---*   
    // | ╱/          |  /            |  ⫽          ⎸  /     
    // |0/           | /             | ⫽           ⎸ /      
    // |∤            ⎹/              ⎹⫽            ⎸/       
    // *-------------* x             0-------------* x

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.10, 0.20, 0.20 }),   //  0 First Segment, First Point
        Coordinate({ 0.70, 0.80, 0.80 }),   //  1 First Segment, Final Point
        Coordinate({ 0.10, 0.10, 0.20 }),   //  2 Second Segment, First Point
        Coordinate({ 0.70, 0.70, 0.80 }),   //  3 Second Segment, Final Point
        Coordinate({ 0.20, 0.10, 0.20 }),   //  4 Third Segment, First Point
        Coordinate({ 0.80, 0.70, 0.80 }),   //  5 Third Segment, Final Point
        Coordinate({ 0.20, 0.10, 0.10 }),   //  6 Fourth Segment, First Point
        Coordinate({ 0.80, 0.70, 0.70 }),   //  7 Fourth Segment, Final Point
        Coordinate({ 0.10, 0.20, 0.10 }),   //  8 Fifth Segment, First Point
        Coordinate({ 0.70, 0.80, 0.70 }),   //  9 Fifth Segment, Final Point
        Coordinate({ 0.70, 0.20, 0.20 }),   // 10 Sixth Segment, First Point
        Coordinate({ 0.10, 0.80, 0.80 }),   // 11 Sixth Segment, Final Point
    };
    mesh.groups.resize(6);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line),
    };
    mesh.groups[2].elements = {
        Element({4, 5}, Element::Type::Line),
    };
    mesh.groups[3].elements = {
        Element({6, 7}, Element::Type::Line),
    };
    mesh.groups[4].elements = {
        Element({8, 9}, Element::Type::Line),
    };
    mesh.groups[5].elements = {
        Element({10, 11}, Element::Type::Line),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),   // 0 First, Second, Third, Fourth and Fifth Segments, First Point and Sixth Segment, Second Point
        Coordinate({ 0.0, 1.0, 0.0 }),   // 1 First and Fifth Segments, Second Point and Sixth Segment, Third Point
        Coordinate({ 0.0, 1.0, 1.0 }),   // 2 First Segment, Third Point and Sixth Segment, Final Point
        Coordinate({ 1.0, 1.0, 1.0 }),   // 3 First, Second, Third, Fourth and Fifth Segments, Final Point
        Coordinate({ 0.0, 0.0, 1.0 }),   // 4 Second Segment, Second Point
        Coordinate({ 1.0, 0.0, 1.0 }),   // 5 Second and Third Segments, Third Point
        Coordinate({ 1.0, 0.0, 0.0 }),   // 6 Third, Fourth and Fifht Segments, Second Point and Sixth Segment, First Point
        Coordinate({ 1.0, 1.0, 0.0 }),   // 7 Fourth and Fifth Segments, Third Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
        },
        {
            Element({0, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
            Element({5, 3}, Element::Type::Line),
        },
        {
            Element({0, 6}, Element::Type::Line),
            Element({6, 5}, Element::Type::Line),
            Element({5, 3}, Element::Type::Line),
        },
        {
            Element({0, 6}, Element::Type::Line),
            Element({6, 7}, Element::Type::Line),
            Element({7, 3}, Element::Type::Line),
        },
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 7}, Element::Type::Line),
            Element({7, 3}, Element::Type::Line),
        },
        {
            Element({6, 0}, Element::Type::Line),
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 3);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        EXPECT_TRUE(resultGroup.elements[1].isLine());
        EXPECT_TRUE(resultGroup.elements[2].isLine());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}



TEST_F(StructurerTest, transformSingleSegmentsIntoNodes)
{

    // *-------------*-------------*          *---------{2|3->1}----------* 
    // |         2   |             |          |             |             | 
    // |         |   |             |          |             |             | 
    // |         3   |             |  ->      |             |             | 
    // |   1         |             |          |             |             | 
    // |  /          |  _6         |          |             |             | 
    // | 0   4-------5-‾           |          |             |             | 
    // *-------------*-------------*       {0|1->0}======={5->3}----------* 
    //                                      {4->2}       {5|6->3}

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1, 0.2, 0.3 }), // 0 First Segment, First Point
        Coordinate({ 0.3, 0.3, 0.3 }), // 1 First Segment, Final Point
        Coordinate({ 0.7, 0.9, 0.8 }), // 2 Second Segment, First Point
        Coordinate({ 0.7, 0.6, 0.8 }), // 3 Second Segment, Final Point
        Coordinate({ 0.4, 0.1, 0.7 }), // 4 Third Segment, First Point
        Coordinate({ 1.0, 0.1, 0.7 }), // 5 Third Segment, Second Point
        Coordinate({ 1.4, 0.3, 0.9 }), // 6 Third Segment, Final Point
    };

    mesh.groups.resize(3);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({4, 5}, Element::Type::Line),
        Element({5, 6}, Element::Type::Line),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }), // 0 First Segment, Only Point
        Coordinate({ 1.0, 1.0, 1.0 }), // 1 Second Segment, Only Point
        Coordinate({ 0.0, 0.0, 1.0 }), // 2 Third Segment, First Point
        Coordinate({ 1.0, 0.0, 1.0 }), // 3 Third Segment, Final Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0}, Element::Type::Node),
        },
        {
            Element({1}, Element::Type::Node),
        },
        {
            Element({2, 3}, Element::Type::Line),
            Element({3}, Element::Type::Node),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    ASSERT_EQ(resultMesh.groups[0].elements.size(), 1);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 1);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 2);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }

    EXPECT_TRUE(resultMesh.groups[0].elements[0].isNode());
    EXPECT_TRUE(resultMesh.groups[1].elements[0].isNode());
    ASSERT_TRUE(resultMesh.groups[2].elements[0].isLine());
    EXPECT_TRUE(resultMesh.groups[2].elements[1].isNode());

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        for (std::size_t e = 0; e < resultGroup.elements.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedElements[g][e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}



TEST_F(StructurerTest, transformGroupsWithMultipleLines)
{

    // *-------------*-------------*          *-------------*----------{2->3} 
    // |             |             |          |             |             ║ 
    // |             |             |          |             |             ║ 
    // |             |        _2   |  ->      |             |             ║ 
    // |             |     _-‾     |          |             |             ║ 
    // |             |  _-‾        |          |             |             ║ 
    // |     0-------1-‾           |          |             |             ║ 
    // *-------------*-------------*          0=============1=========={1.5->2}
    //                                        

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.4, 0.1, 0.7 }), // 0 First Segment, First Point
        Coordinate({ 1.0, 0.1, 0.7 }), // 1 First Segment, Second Point
        Coordinate({ 1.8, 0.6, 0.7 }), // 2 First Segment, Final Point
    };

    mesh.groups.resize(1);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 1.0 }), // 0 First Segment, First Point
        Coordinate({ 1.0, 0.0, 1.0 }), // 1 First Segment, Final Point, Second Segment, First Point
        Coordinate({ 2.0, 0.0, 1.0 }), // 2 Second Segment, Middle Point
        Coordinate({ 2.0, 1.0, 1.0 }), // 3 Second Segment, Final Point
    };

    Elements expectedElements = {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), 1);
    ASSERT_EQ(resultMesh.groups[0].elements.size(), expectedElements.size());

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }

    ASSERT_TRUE(resultMesh.groups[0].elements[0].isLine());
    ASSERT_TRUE(resultMesh.groups[0].elements[1].isLine());
    ASSERT_TRUE(resultMesh.groups[0].elements[2].isLine());

    for (std::size_t e = 0; e < expectedElements.size(); ++e) {
        auto& resultElement = resultMesh.groups[0].elements[e];
        auto& expectedElement = expectedElements[e];

        for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
            EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
        }
    }
}



TEST_F(StructurerTest, transformTriangleIntoStructuredSurface)
{

    // y                y                         y                y
    // *----------*     {2->3}====={1->2}         *----------*                1==========2
    // |      __1 |        ║\\\\\\\\\\║           |    __->2 |                ║//////////║
    // | 2<-‾‾ ╱  |        ║\\\\\\\\\\║           | 1--   /  |                ║//////////║
    // |  \   ╱   |   ->   ║\\\\\\\\\\║           |  \   ╱   |     ->         ║//////////║
    // |    0     |        ║\\\\\\\\\\║           |    0     |                ║//////////║
    // *----------* x      0======{0.5->1} x      *----------* x              0======(2.5->3) x

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.4, 0.2, 0.0 }),    // 0 First Triangle, First Point, Second Triangle, Third Point, X-Y Plane
        Coordinate({ 0.9, 0.9, 0.0 }),    // 1 First and Second Triangles, Second Point, X-Y Plane
        Coordinate({ 0.2, 0.6, 0.0 }),    // 2 First Triangle, Third Point, Second Triangle, First Point, X-Y Plane
    };
    mesh.groups.resize(1);
    mesh.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface),
        Element({2, 1, 0}, Element::Type::Surface),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First Quad, First Point, Second Quad, Fourth Point X-Y Plane
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First Quad, Second Point, Second Quad, Third Point, X-Y Plane
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First Quad, Third Point, Second Quad, Second Point, X-Y Plane
        Coordinate({ 0.0, 1.0, 0.0 }),    // 3 First Quad, Fourth Point, Second Quad, First Point, X-Z Plane
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1, 2, 3}, Element::Type::Surface),
            Element({3, 2, 1, 0}, Element::Type::Surface),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 2);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isQuad());
        EXPECT_TRUE(resultGroup.elements[1].isQuad());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}



TEST_F(StructurerTest, transformTriangleIntoNode)
{

    //     *-------------*               *-------------*
    //    /|            /|              /|            /|    
    //   / |           ╱ |             / |           / |    
    // z/  |          ╱  |           z/  |          /  |    
    // *---2---------*   |    ->     *---┼---------*   |    
    // |  ||y⟍       ⎸   ⎸           ⎸   ⎸ y       ⎸   ⎸    
    // | ⎹ *-_=1-----┼---*           | {0|1|2}-----┼---*   
    // | 0/-‾        |  /            |  /          |  /     
    // | /           | /             | /           | /      
    // |/            |/              |/            |/       
    // *-------------* x             0-------------* x  

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.2, 0.6, 0.2 }),    // 0 First Triangle, First Point
        Coordinate({ 0.4, 0.9, 0.2 }),    // 1 First Triangle, Second Point
        Coordinate({ 0.2, 0.8, 0.4 }),    // 2 First Triangle, Third Point
    };
    mesh.groups.resize(1);
    mesh.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface),
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 1.0, 0.0 }),    // 0 All Nodes' Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0}, Element::Type::Node),
            Element({0}, Element::Type::Node),
            Element({0}, Element::Type::Node),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 3);
    }
    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isNode());

        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}



TEST_F(StructurerTest, transformTriangleIntoLines)
{
    //     *-------------*                2-------------*           *-------------*              {4->2}----------*        
    //    /|            /|               /⦀            /|          /|            /|               /⦀            /⎹        
    //   /┄|┄┄┄2       ╱ |              / ⫼           / |         / |  4        ╱ |              / ⦀           / ⎹        
    // z/  |  ⎹|      ╱  |            z/  ⦀          /  |       z/  |  ⎸⎸      ╱  |            z/  ⦀          /  ⎹        
    // *---┼--┼┼-----*   |     ->     *---⫵---------*   ⎸       *---┼-┼-┼----*    ⎸     ->     *---⫵---------*   ⎸       
    // |   |y⎹ |     |   |            ⎹   ⦀ y       |   |       |   |y|  ⎸    ⎸   |            ⎹   ⫵         ⎹   |        
    // |  ┄*┄⎸┄1-----┼---*            ⎹   1---------┼---*       |   *-⎸--⎹-⋰--┼---*            ⎹{3.5->1}≡≡≡≡≡≡╪{5->3}        
    // |  / ⎹ ⟋      ⎸  /             |  ⫻         |  /        |  / ⎹  _-5   |  /             |  ⫻          |  /      
    // | ┄┄┄0        | /              ⎹ ⫻          ⎹ /         ⎹ ┄┄┄3-‾      ⎹ /              ⎹ ⫻           ⎹ /       
    // |/            |/               ⎹⫻           ⎹/          ⎹/            ⎹/               ⎹⫻            ⎹/        
    // *-------------* x              0-------------* x         *-------------* x            {3->0}-----------* x    

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.4, 0.2, 0.2 }),    // 0 First Triangle, First Point
        Coordinate({ 0.4, 0.9, 0.2 }),    // 1 First Triangle, Second Point
        Coordinate({ 0.4, 0.9, 0.6 }),    // 2 First Triangle, Third Point
        Coordinate({ 0.2, 0.4, 0.2 }),    // 3 Second Triangle, First Point
        Coordinate({ 0.2, 0.8, 0.8 }),    // 4 Second Triangle, Second Point
        Coordinate({ 0.6, 0.8, 0.2 }),    // 5 Second Triangle, Third Point
    };
    mesh.groups.resize(2);
    mesh.groups[0].elements = { Element({0, 1, 2}, Element::Type::Surface) };
    mesh.groups[1].elements = { Element({3, 4, 5}, Element::Type::Surface) };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First and Fourth Edges, First Point, Third and Sixth Edges, Final Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 1 First Edge, Final Point, Second Edge, First Point, Third, Fourth, Fifth and Sixth Edges, Second Point
        Coordinate({ 0.0, 1.0, 1.0 }),    // 2 Second and Fourth Edge, End Point, Third and Fifth Edges, First Point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 3 Fifth Edge, Final Point, Sixth Edge, First Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
            Element({1, 3}, Element::Type::Line),
            Element({3, 1}, Element::Type::Line),
            Element({1, 0}, Element::Type::Line),
        }
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    EXPECT_EQ(resultMesh.groups[0].elements.size(), 4);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 6);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];


        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];
            EXPECT_TRUE(expectedElement.isLine());

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}

TEST_F(StructurerTest, transformTriangleIntoSurfacesAndLines)
{
    //     *-------------*                 *-----------{2->3}         *-------------*                *------------{1->3}
    //    /|            /|                /|            ⫽║           /|            /|               /|             /⦀
    //   / |           ╱_2               / |           ⫽/║          / |           / |              / |            / ⦀
    // z/  |         _/‾||             z/  |          ⫽//║        z/  |          ⌿-1⎹            z/  ⎹           /  ⎻⦀
    // *---┼------=-‾* | |      ->     *---┼----{1.5->2}/⎻║        *---┼------_-‾* ⎹┆|      ->    *---┼-----{1.5->2} ⦀
    // |   |y  _-‾   | ⎸ |             ⎹   | y       ║///⎻║        |   |y  _-‾   | ⎸┆⎹            |   | y        |   ⦀
    // |   *_-=------┼┼--*             ⎹   *---------╫{3.5->4}    ⎹   *_-=----- ┼⎹-┴*            ⎹{0.33->1}===⎻==╪{0.66->2}
    // | _-‾    __ --1  /              ⎹  /          ║//⫽         | 0⌿=-___     ⎹| /             ⎹  ⫽///////////|//⫽
    // ├0-- ‾‾       | /               ⎹ /           ║/⫽          | ∤      ‾‾‾--2 /              ⎹ ⫽////////////|/⫽
    // |/            |/                ⎹/            ║⫽           |/            |/               ⎹⫽/////////////|⫽
    // *-------------* x               0≡≡≡≡≡≡≡≡≡≡≡≡≡1 x          *-------------* x               0============{2->4} x

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.3, 0.0, 0.1 }),    // 0 First Triangle, First Point
        Coordinate({ 1.0, 0.0, 0.4 }),    // 1 First Triangle, Second Point
        Coordinate({ 1.0, 1.0, 0.8 }),    // 2 First Triangle, Third Point
        Coordinate({ 0.0, 0.4, 0.2 }),    // 3 Second Triangle, First Point
        Coordinate({ 0.9, 0.8, 0.6 }),    // 4 Second Triangle, Second Point
        Coordinate({ 1.0, 0.0, 0.3 }),    // 5 Second Triangle, Third Point
    };
    mesh.groups.resize(2);
    mesh.groups[0].elements = { Element({0, 1, 2}, Element::Type::Surface) };
    mesh.groups[1].elements = { Element({3, 4, 5}, Element::Type::Surface) };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First Edge and Second Quad, First Point, Second Edge, Final Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First Edge, Final Point, First Quad, First Point, Second Edge, First Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 2 First Quad, Second Point, Second Quad, Fourth Point
        Coordinate({ 1.0, 1.0, 1.0 }),    // 3 First Quad, Third Point, Third Edge, Final Point, Fourth Edge, First Point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 4 First Quad, Fourth Point, Second Quad, Third Point, Third Edge, First Point and Fourth Edge, Final Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 5 First Quad, Second Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2, 3, 4}, Element::Type::Surface),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({0, 5, 4, 1}, Element::Type::Surface),
            Element({4, 3}, Element::Type::Line),
            Element({3, 4}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    EXPECT_EQ(resultMesh.groups[0].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 3);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    EXPECT_TRUE(resultMesh.groups[0].elements[0].isLine());
    EXPECT_TRUE(resultMesh.groups[0].elements[1].isQuad());
    EXPECT_TRUE(resultMesh.groups[0].elements[2].isLine());
    EXPECT_TRUE(resultMesh.groups[1].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[1].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[1].elements[2].isLine());


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];


        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}

TEST_F(StructurerTest, transformTriangleIntoTwoSurfaces)
{
    //     *-------------*                 *-----------{2->3}               *------------⎻*                  *---------------0    
    //    /|            /|                /|             ⫽║                /|            /⎸                /|             ⫽⎻║      
    //   / |           /⎽⎼2               / |            ⫽/║               / |           /⎽⎼0               / |            ⫽\⎻║     
    // z/  |          /⎻⎺||             z/  |           ⫽//║             z/  |          /⎻⎺⎹|             z/  |           ⫽\\⎻║     
    // *---┼--------⎽⎼* | |      ->     *---┼-----{1.5->2}/⎻║             *---┼--------⎽⎼*  ||      ->     *---┼-----{0.5->1}\\║    
    // |   |y    ⎽⎼⎻⎺  | ⎸ |             |   | y        ⎻║///║             ⎹   |y    ⎽⎼⎻⎺  | | |             ⎹   ⎹ y         ║\\\║   
    // |   *---⎽⎼⎻⎺----┼┼--*             |{2.66->5}=====⎻║{2.33->4}        ⎹   *---⎽⎼⎻⎺----┼┼--*             ⎹{2.33->4}======║{2.66->5}
    // |  / ⎽⎼⎻⎺    __-1  /              |  ⫽\\\\\\\\\\\║//⫽              |  / ⎽⎼⎻⎺    __-1  /              ⎸  ⫽///////////║\\⫽     
    // | ⎽⎼⎻⎺ __--‾‾   | /               | ⫽\\\\\\\\\\\\║/⫽               | /⎽⎼⎻⎺__--‾‾   ⎸ /               ⎸ ⫽////////////║\⫽      
    // 0⌿-‾‾         ⎹/                ⎹⫽\\\\\\\\\\\\\║⫽                2⌿-‾‾         ⎹/                ⎹⫽/////////////⎻║⫽         
    // *-------------* x               0==============1 x                *-------------* x             {2->3}========={1->2} x

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.0, 0.0, 0.1 }),    // 0 First, First Point, Second Triangle, Third Point
        Coordinate({ 1.0, 0.0, 0.4 }),    // 1 First and Second Triangles, Second Point
        Coordinate({ 0.9, 1.0, 0.8 }),    // 2 First Triangle, Third Point, Second Triangle, First Point
    };
    mesh.groups.resize(2);
    mesh.groups[0].elements = { Element({0, 1, 2}, Element::Type::Surface) };
    mesh.groups[1].elements = { Element({2, 1, 0}, Element::Type::Surface) };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First Quad, First Point, Fourth Quad, Second Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First Quad, Second Point, Second and Fourth Quads, First Point, Third Quad, Third Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 2 Second and Third Quads, Second Point
        Coordinate({ 1.0, 1.0, 1.0 }),    // 3 Second Quad, Third Point, Third Quad, First Point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 4 First Quad, Third Point, Second, Third and Fourth Quads, Fourth Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 5 First Quad, Fourth Point, Fourth Quad, Third Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1, 4, 5}, Element::Type::Surface),
            Element({1, 2, 3, 4}, Element::Type::Surface),
        },
        {
            Element({3, 2, 1, 4}, Element::Type::Surface),
            Element({1, 0, 5, 4}, Element::Type::Surface),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    EXPECT_EQ(resultMesh.groups[0].elements.size(), 2);
    EXPECT_EQ(resultMesh.groups[1].elements.size(), 2);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    EXPECT_TRUE(resultMesh.groups[0].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[0].elements[1].isQuad());
    EXPECT_TRUE(resultMesh.groups[1].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[1].elements[1].isQuad());


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];


        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}

TEST_F(StructurerTest, transformTriangleWithEquidistantEdges)
{
    // T0   *-------------1                  *-----------{1->0}          T2      2-------------*               {2->3}-----------*
    //     /|           ⫽⎻║                 /⎹             /║                   /║\           /|                / ║            /|
    //    / |          ╱/⎹|                / ⎹            / ║                  /⎹| \         / |               /  ║           / |
    //  z/  |        ╱ / ||              z/  ⎹           /  ║               z / ⎸|  \       /  |             z/   ║          /  |
    //  *---┼------⌿--*  |⎹       ->     *---⎻┼----------*   ║                *-┼-┼---------*   ⎸      ->     *----╫---------*   |
    //  |   |y   ╱    | ⎹ |              ⎸   ⎹     ->   |   ║                | ⎸ |y   \    |   |             ⎹    ║ y   ->  |   |
    //  |   *--╱------┼-┼-*              ⎸{0.33->4}=====╪{1.5|0.66->1}       | ⎸ *-----\---┼---*            {1.5|2.33->2}==={2.66->4}
    //  |  / ╱        |⎹ /               ⎸⩘ ⫽//////////⎹///⫽                ⎸⎸ /       \  ⎸  /              |⩘ ⫽//////////|//⫽
    //  | /╱          |⎸/                ⎸/⫽////////////⎸/⫽ /               |⎸/         \ | /               ⎹/⫽////////////⎸/⫽/ 
    //  |⫽           ⎻║/                 ⎸⫽/////////////⎸⫽ ⩗                ║/           \⎸/                |⫽////////////⎹⫽ ⩗ 
    //  0=============2 x             {0->3}============2 x                  1=============0 x               1==============0 x
    //                                         <-                                                                  <-
    // 
    // T1   *-------------1                  *-----------{1->2}         T3      0-------------*                 0--------------*
    //     /|           ⫽⎻║                 /⎹             /║                  /║\           /|                / ║            /|
    //    / |          ╱/⎹|                / ⎹            / ║                 /⎹| \         / |               /  ║           / |
    //  z/  |        ╱ / ||              z/  ⎹           /  ║              z / ⎸|  \       /  |            z /   ║          /  |
    //  *---┼------⌿--*  |⎹       ->     *---⎻┼----------*   ║               *-┼-┼---------*   ⎸      ->     *----╫---------*   |
    //  |   |y   ╱    | ⎹ |              |   ⎹     <-   |   ║               | ⎸ |y   \    |   |             |    ║ y   <-  |   |
    //  |   *--╱------┼-┼-*              |{1.66->4}=====╪{0.5|1.33->1}      |⎹  *-----\---┼---*            {0.5|2.66->1}==={2.33->4}
    //  |  / ╱        |⎹ /               | /⫽\\\\\\\\\\\⎸\\⫽               || /       \  ⎸  /              |  /⫽\\\\\\\\\\⎸\\\⫽
    //  | /╱          |⎸/                |⩗⫽\\\\\\\\\\\⎹\⫽⩘               ⎹⎸/         \ | /               ⎹ ⩗⫽\\\\\\\\\\\⎸\\⫽⩘
    //  |⫽           ⎻║/                 |⫽/\\\\\\\\\\\\⎸⫽/                ║/           \⎸/                |⫽\\\\\\\\\\\\\⎸⫽ /
    //  2=============0 x             {2->3}============0 x                 1=============2 x             {1->2}=========={2->3} x
    //                                          ->                                                                  ->
    // 
    // T4    *------------*                 *-----------{1->4}           T6      1------------*               {1->0}====={0.66|1.5->1}
    //      /|           /|                /|             ⫽⎻║                   /\⎺⎻⎼⎽         /⎹                 /|             ⫽║
    //     / |          / |               / |          ⩘ ⫽/║                  / |\ ⎺⎻⎼⎽      /  ⎸               / |         ⩘  ⫽/║
    //  z /  |         /  |             z/  |         / ⫽//⎻║ |              z/  | \   ⎺⎻⎼⎽  /   ⎸             z/  |         / ⫽//║ ⎸
    //   1==_|--------*   |       -> {1->2}=╪==={0.5|1.33->1}v               *---┼--\----⎺⎻*⎽   ⎹       ->    *--⎻-┼-----{0.33->4}⎻║ v
    //   |⎺⎻⎼⎽ |‾‾‾--___|   |             ⎹   |    ->    ⎻║////║                ⎸   ⎸   \    ⎹ ⎺⎻⎼⎽ ⎸            ⎹    ⎸         ⎻║////║
    //   |  ⎺⎻⎼⎽--------|‾==2             ⎹   *---------⎻-║/{2->3}              ⎸   *----\---⎻┼---2             ⎸   *----------║////2
    //   |  / ⎺⎻⎼⎽      |  ⫽              ⎸  /         ᐱ ║//⫽                 |  /       \  ⎸  ⫽             |  /         ᐱ ║///⫽
    //   | /     ⎺⎻⎼⎽   | ⫽               ⎸ /          ⎹ ║/⫽ /                ⎹ /         \ | ⫽              ⎹ /          ⎹ ⎻║//⫽ /
    //   |/         ⎺⎻⎼⎽|⫽                ⎸/             ║⫽ ⩗                 |/           \⎸⫽               |/             ║⫽ ⩗
    //   *------------0 x               *--------------0 x                   *-------------0 x               *-------------{0->3} x
    //                          
    // 
    // T5   *------------*                 *----------{1.33->4}          T7      2--------------*               {2->3}====={2.33|1.5->2}
    //     /|           /|                /|             ⫽⎻║                     /\⎺⎻⎼⎽          / ⎸                /|            ⫽║
    //    / |          / |               / |          / ⫽\⎻║                    /| \  ⎺⎻⎼⎽      /  ⎸               / ⎹         / ⫽/║
    // z /  |         /  |             z/  |         ⩗ ⫽\\║ ᐱ                z/ |  \    ⎺⎻⎼⎽  /   ⎸             z/  |        ⩗⫽//⎺║ ᐱ 
    //  2==_┼--------*   |       ->    2===╪==={2.5|1.66->3}⎹                 *--┼---\-----⎺⎻*    ⎸      ->     *---┼----{2.66->4}⎺║ ⎸
    //  |⎺⎻⎼⎽ |‾‾‾--___|   |             ⎹   |    ->    ⎻║\\\⎻║                  |  ⎹    \     ⎹ ⎺⎻⎼⎽ ⎸             ⎸   |        ⎻║////║
    //  |  ⎺⎻⎼⎽--------|‾==1             ⎹   *----------⎻║\\\1                   ⎸  *-----\----┼---1             |   *---------║////1
    //  |  / ⎺⎻⎼⎽      |  ⫽              ⎸  /         | ║\\⫽                   |  /       \   ⎸  ⫽             ⎹  /        ⎹ ║///⫽
    //  | /     ⎺⎻⎼⎽   | ⫽               ⎸ /          V ║\⫽ ⩘                  ⎸ /         \ ⎹  ⫽              ⎸ /         V ║/⫽⩘
    //  |/         ⎺⎻⎼⎽|⫽                ⎸/             ║⫽ /                   |/            \⎸⫽               |/            ║⫽ /
    //  *------------0 x               *--------------0 x                     *-------------0 x               *-------------0 x
    // 
    //                                            ->                                                            ->
    // T8    *------------*             {1.33->4}===={1.66|2.5->3}      T10      *-----------*          {1.5|2.33->2}===={2.66->4}
    //      /|           /|                /║╱╱╱╱╱╱╱╱╱╱╱╱╱⫻║                   /⎹           /|                ⫽║╱╱╱╱╱╱╱╱╱╱╱╱╱⫽║
    //     / |          / |               /ᐱ║╱╱╱╱╱╱╱╱╱╱╱╱⫻⎻║                  / ⎹          / ⎹               ⫽ᐱ║╱╱╱╱╱╱╱╱╱╱╱╱⫽╱║
    //  z /  |         /  |             z/ |║///////////⫻╱╱║ |               /  ⎹         /  |             z⫽ |║///////////⫽╱╱║ |
    //   0---┼-------=2   |       ->    *---║⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿2╱╱╱║ v               2==__-------*   |     ->   {2->3}-⎻║⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿⌿*╱╱/║ v
    //   |   | __--‾‾ | ⎺⎻⎼⎽|             ⎸   ║╱╱╱╱╱╱╱╱╱╱⎹╱╱╱╱║                ⎹ ⎺⎻⎼⎽⎹‾‾‾--___|   |             ⎹   ⎻║╱╱╱╱╱╱╱╱╱╱⎹╱╱╱⎻║
    //   |   1≡=======╪===0             |   1==========╪====0                |   1========|‾‾≡0             |   1==========╪====0
    //   |  /         |  /              |  /      <-   |   /                 |  /         |  /              |  /      <-   |   /
    //   | /          | /               | /            |  /                  | /          | /               | /            |  /
    //   |/           |/                |/             | /                   |/           |/                |/             | /
    //   *------------* x               *--------------* x                   *------------* x               *--------------* x
    //                                                                                                           
    //                                            <-                                                              <-
    // T9    *------------*             {1.66->4}===={0.5|1.33->1}      T11       *------------*          {1.5|0.33->1}===={0.33->4}
    //      /|           /|                /║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍⫽║                    /|           /|                ⫽⎻║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍/║
    //     / |          / |               /|║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍⫽⟍║                   / |          / |               ⫽|⎻║⟍⟍⟍⟍⟍⟍⟍⟍⟍/⟍║
    // z  /  |         /  |             z/ v║⟍⟍⟍⟍⟍⟍⟍⟍⟍⫽⟍⟍║ ᐱ              z /  ⎸         /  ⎸            z⫽  v║⟍⟍⟍⟍⟍⟍⟍⟍/⟍⟍║ ᐱ
    //   0---┼-------=1   |       ->    *---║⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀{1->2}⟍⎻║ ⎹               1==__--------*   |     ->   {1->0}-⎻║⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀⍀*⟍⟍⎻║ |
    //   |   | __--‾‾ | ⟍ ⎸             ⎸   ║⟍⟍⟍⟍⟍⟍⟍⟍|\⟍⟍⎻║                 | ⎺⎻⎼⎽|‾‾‾--___|   |             ⎹   ⎻║⟍⟍⟍⟍⟍⟍⟍⟍|⟍⟍⎻║
    //   |   2≡=======╪===0             |{2->3}========╪====0                 |   2========|‾‾≡0             |   2==========╪={0->3}
    //   |  /         |  /              |  /      ->   |   /                  |  /         |  /              |  /      ->   |  /
    //   | /          | /               | /            |  /                   | /          | /               | /            | /
    //   |/           |/                |/             | /                    |/           |/                |/             |/
    //   *------------* x               *--------------* x                    *------------* x               *--------------* x
    //                                                                   
    // 
    // T12   *------------*                  *-------------*           T14       *-------------1                 *-----------{1->2}     
    //      /|           / ⎸                /|            /|                    /|           ⎽⎼⫽|                /|            /║
    //     / |          /  ⎸               / |           / |                   / |        ⎽⎼⎻⎺╱/  ⎸              /  ⎸           /⎻║
    //  z /  |         /   ⎸             z/  |          /  |               z  /  |     ⎽⎼⎻⎺  ╱/   ⎸            z/   ⎸          / ⎻║
    //   *---┼-------=2    ⎸      ->    *--------------2   |                 *---┼---⎽-⎺--⌿--*   |     ->     *----┼----------*  ║
    //   |   | __--‾‾⎼⎻|    ⎸            |    |    ->   ║   |                 ⎸   ⎸⎽⎼⎻⎺   ╱   |    |            ⎹    |    ->    ⎸  ║
    //   |   1=---⎽⎼⎻⎺--┼---*             |    1=========║{1.33->4}            ⎸   0---╱-----┼----*            ⎹    0========={0.5|1.33->1}
    //   |  ⫽  ⎽⎼⎻⎺     ⎸  /              ⎸⩘ ⫽/////////⎼║//⫽                  ⎸  ⫽ ╱       ⎹   /              ⎸⩘ ⫽//////////|//⫽
    //   | ⫽ ⎽⎼⎻⎺       ⎸ /               ⎸/⫽///////////║/⫽ /                 ⎸ ⫽╱         ⎹  /               ⎸/⫽///////////⎹/⫽ /
    //   |⫽⎽⎼⎻⎺         ⎸/                ⎸⫽////////////║⫽ ⩗                 ⎹⫻            ⎸/                |⫽////////////⎹⫽ ⩗
    //   0------------* x               0========{2.5|1.66->3} x             2-------------* x            {2->3}========{1.66->4} x
    //                                          <-                                                          
    // 
    // 
    // 
    // T13   *------------*                  *--------------*           T15      *-------------2                 *------------2     
    //      /|           / ⎸                /|             /|                   /|           ⎽⎼⫽⎸                /|           /║
    //     / |          /  ⎸               / |            / |                  / |        ⎽⎼⎻⎺╱/ ⎹               /  ⎸          /⎻║
    //  z /  |         /   ⎸             z/  |           /  |               z /  |     ⎽⎼⎻⎺ ╱ /  ⎹             z/   ⎸         / ⎻║
    //   *---┼-------=1    ⎸      ->    *----┼--------{1->2}|                *---┼---⎽-⎺--⌿-*   ⎹      ->     *----┼--------*   ║
    //   |   |y__--‾‾⎼⎻|    ⎸            |    |    <-    ║   |                |   |⎽⎼⎻⎺   ╱   ⎸   ⎹             |    |    <-  ⎸   ║
    //   |   2=---⎽⎼⎻⎺--┼---*             |  {2->3}=======║{1.66->4}           |   0---╱---⎻-┼----*            ⎹     0=======╪{2.5|1.66->3}
    //   |  ⫽  ⎽⎼⎻⎺     ⎸  /              ⎸ / ⫽\\\\\\\\\\║\\⫽ ⩘              ⎹   ⫽ ╱      ⎹   /              ⎸  /⫽\\\\\\\\|\\⫽ ⩘
    //   | ⫽ ⎽⎼⎻⎺       ⎸ /               ⎸⩗⫽\\\\\\\\\\\⎻║\⫽  /               | ⫽╱         | /               ⎹ ⩗⫽\\\\\\\\\|\⫽ /
    //   |⫽⎽⎼⎻⎺         ⎸/                ⎸⫽\\\\\\\\\\\\\║⫽                   |⫻           ⎸/                |⫽\\\\\\\\\\\|⫽ 
    //   0------------* x               0========{0.5|1.33->1} x             1-------------* x               1========{1.33->4} x
    //                                          ->                                                                 ->
    // 
    //                                             <-                                                                  
    // T16  *-------------*          {2.5|1.66->3}====={1.33->4}         T18     *-------------*                   *------------*    
    //     /|            /|                /⫽\\\\\\\\\\\\⫽⎹                    /|            /|                  /|           /⎹    
    //    / |           / |               ⩗⫽\\\\\\\\\\\\⫽⩘⎸                  / ⎹           / ⎹                 / ⎹          /  ⎸   
    //  z/  |          /  |              z⫽\\\\\\\\\\\\⫽ /⎹                 z/  |          /  |               z/  | ->      /  ⎹   
    //  0============≡1   |       ->     0============1    ⎹                 1===╪---------*   |      ->       1===╧==={1.33->4}⎹   
    //  | ⎺⎻⎼⎽|y__--‾‾  |   |              |   ║    ->  |    ⎹                 ║  y|‾‾‾---___|   |             ᐱ ║╱╱╱╱╱╱╱╱╱╱╱╱║   |
    //  |   2=--------┼---*              |   2--------┼----*                 ║   *---------╪=≡≡2             | ║╱╱╱╱╱╱╱╱╱╱╱╱╟---2    
    //  |  /          |  /               |  /         |   /                  ║  /        _-┼‾ /                ║╱╱╱╱╱╱╱╱╱╱╱╱║| ⫽     
    //  | /           | /                | /          |  /                   ║ /    __-‾‾  | /                 ║╱╱╱╱╱╱╱╱╱╱╱╱║v⫽      
    //  |/            |/                 |/           | /                    ║/ _--‾       |/                  ║╱╱╱╱╱╱╱╱╱╱╱╱║⫽       
    //  *-------------* x                *------------* x                    0=‾-----------* x                 0======{2.5|1.66->3} x      
    //                                                                                                               <-
    //                                                                    
    //                                             <-
    // T17  *-------------*          {0.5|1.33->1}====={1.66->4}          T19     *-------------*                   *------------*
    //     /|            /|                ⩘⫽\\\\\\\\\\\\⫽⎸                     /⎸            /⎸                  /⎸           /|
    //    / |           / |               /⫽\\\\\\\\\\\\⫽/|                    / |           / |                 / |          / ⎹
    //  z/  |          /  |              z⫽\\\\\\\\\\\\⫽⩗ ⎸                  z/  ⎸          /  ⎸               z/  ⎸ <-      /  |
    //  0============≡2   |       ->     0=========={2->3} |                  2===╪---------*   |      ->     {2->3}╧==={1.66->4}⎹
    //  | ⎺⎻⎼⎽|y__--‾‾  |   |              |   ║    ->  ⎹    |                  ║  y|‾‾‾---___|   |               ║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍║   ⎹
    //  |   1=--------┼---*              |{1->2}------┼----*                  ║   *---------╪=≡≡1             | ║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍╟-{1->2}
    //  |  /          |  /               |  /         ⎹   /                   ║  /        _-┼‾ /              v ║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍║ᐱ ⫽
    //  | /           | /                | /          ⎹  /                    ║ /    __-‾‾  | /                 ║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍║|⫽
    //  |/            |/                 |/           ⎹ /                     ║/ _--‾       |/                  ║⟍⟍⟍⟍⟍⟍⟍⟍⟍⟍║⫽
    //  *-------------* x                *------------* x                     0=‾-----------* x                 0======{0.5|1.33->1} x
    //                                         <-                                                                      ->

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1, 0.1, 0.1 }),    // 0
        Coordinate({ 0.9, 0.9, 0.9 }),    // 1
        Coordinate({ 0.9, 0.1, 0.1 }),    // 2
        Coordinate({ 0.1, 0.9, 0.9 }),    // 3
        Coordinate({ 0.1, 0.1, 0.9 }),    // 4
        Coordinate({ 0.9, 0.9, 0.1 }),    // 5
        Coordinate({ 0.1, 0.9, 0.1 }),    // 6
        Coordinate({ 0.9, 0.1, 0.9 }),    // 7
    };
    mesh.groups.resize(20);
    mesh.groups[0].elements =  { Element({0, 1, 2}, Element::Type::Surface) };
    mesh.groups[1].elements =  { Element({2, 1, 0}, Element::Type::Surface) };
    mesh.groups[2].elements =  { Element({2, 0, 3}, Element::Type::Surface) };
    mesh.groups[3].elements =  { Element({3, 0, 2}, Element::Type::Surface) };
    mesh.groups[4].elements =  { Element({2, 4, 5}, Element::Type::Surface) };
    mesh.groups[5].elements =  { Element({2, 5, 4}, Element::Type::Surface) };
    mesh.groups[6].elements =  { Element({2, 3, 5}, Element::Type::Surface) };
    mesh.groups[7].elements =  { Element({2, 5, 3}, Element::Type::Surface) };
    mesh.groups[8].elements =  { Element({5, 6, 7}, Element::Type::Surface) };
    mesh.groups[9].elements =  { Element({5, 7, 6}, Element::Type::Surface) };
    mesh.groups[10].elements = { Element({5, 6, 4}, Element::Type::Surface) };
    mesh.groups[11].elements = { Element({5, 4, 6}, Element::Type::Surface) };
    mesh.groups[12].elements = { Element({0, 6, 7}, Element::Type::Surface) };
    mesh.groups[13].elements = { Element({0, 7, 6}, Element::Type::Surface) };
    mesh.groups[14].elements = { Element({6, 1, 0}, Element::Type::Surface) };
    mesh.groups[15].elements = { Element({6, 0, 1}, Element::Type::Surface) };
    mesh.groups[16].elements = { Element({4, 7, 6}, Element::Type::Surface) };
    mesh.groups[17].elements = { Element({4, 6, 7}, Element::Type::Surface) };
    mesh.groups[18].elements = { Element({0, 4, 5}, Element::Type::Surface) };
    mesh.groups[19].elements = { Element({0, 5, 4}, Element::Type::Surface) };

    Coordinates expectedCoordinates = {
        Coordinate({ 1.0, 1.0, 1.0 }),    // 0
        Coordinate({ 1.0, 1.0, 0.0 }),    // 1
        Coordinate({ 1.0, 0.0, 0.0 }),    // 2
        Coordinate({ 0.0, 0.0, 0.0 }),    // 3
        Coordinate({ 0.0, 1.0, 0.0 }),    // 4
        Coordinate({ 0.0, 1.0, 1.0 }),    // 5
        Coordinate({ 1.0, 0.0, 1.0 }),    // 6
        Coordinate({ 0.0, 0.0, 1.0 }),    // 7
    };

    std::vector<Elements> expectedElements = {
        {
            Element({1, 2, 3, 4}, Element::Type::Surface),
            Element({0, 1}, Element::Type::Line),
        },
        {
            Element({2, 1, 4, 3}, Element::Type::Surface),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({2, 3, 4, 1}, Element::Type::Surface),
            Element({4, 5}, Element::Type::Line),
        },
        {
            Element({5, 4}, Element::Type::Line),
            Element({4, 3, 2, 1}, Element::Type::Surface),
        },
        {
            Element({2, 6, 0, 1}, Element::Type::Surface),
            Element({6, 7}, Element::Type::Line),
        },
        {
            Element({2, 1, 0, 6}, Element::Type::Surface),
            Element({7, 6}, Element::Type::Line),
        },
        {
            Element({0, 1, 2, 6}, Element::Type::Surface),
            Element({5, 0}, Element::Type::Line),
        },
        {
            Element({2, 1, 0, 6}, Element::Type::Surface),
            Element({0, 5}, Element::Type::Line),
        },
        {
            Element({1, 4, 5, 0}, Element::Type::Surface),
            Element({6, 0}, Element::Type::Line),
        },
        {
            Element({1, 0, 5, 4}, Element::Type::Surface),
            Element({0, 6}, Element::Type::Line),
        },
        {
            Element({1, 4, 5, 0}, Element::Type::Surface),
            Element({5, 7}, Element::Type::Line),
        },
        {
            Element({5, 4, 1, 0}, Element::Type::Surface),
            Element({7, 5}, Element::Type::Line),
        },
        {
            Element({3, 4, 1, 2}, Element::Type::Surface),
            Element({6, 2}, Element::Type::Line),
        },
        {
            Element({3, 2, 1, 4}, Element::Type::Surface),
            Element({2, 6}, Element::Type::Line),
        },
        {
            Element({4, 1, 2, 3}, Element::Type::Surface),
            Element({1, 0}, Element::Type::Line),
        },
        {
            Element({4, 3, 2, 1}, Element::Type::Surface),
            Element({0, 1}, Element::Type::Line),
        },
        {
            Element({7, 6, 0, 5}, Element::Type::Surface),
            Element({4, 5}, Element::Type::Line),
        },
        {
            Element({7, 5, 0, 6}, Element::Type::Surface),
            Element({5, 4}, Element::Type::Line),
        },
        {
            Element({3, 7, 6, 2}, Element::Type::Surface),
            Element({1, 2}, Element::Type::Line),
        },
        {
            Element({3, 2, 6, 7}, Element::Type::Surface),
            Element({2, 1}, Element::Type::Line),
        },
    };

    auto resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());

    EXPECT_EQ(resultMesh.groups[0].elements.size(), 2);
    EXPECT_EQ(resultMesh.groups[1].elements.size(), 2);
    EXPECT_EQ(resultMesh.groups[2].elements.size(), 2);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }


    EXPECT_TRUE(resultMesh.groups[0].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[0].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[1].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[1].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[2].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[2].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[3].elements[0].isLine());
    EXPECT_TRUE(resultMesh.groups[3].elements[1].isQuad());
    EXPECT_TRUE(resultMesh.groups[4].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[4].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[5].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[5].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[6].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[6].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[7].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[7].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[8].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[8].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[9].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[9].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[10].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[10].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[11].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[11].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[12].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[12].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[13].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[13].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[14].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[14].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[15].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[15].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[16].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[16].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[17].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[17].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[18].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[18].elements[1].isLine());
    EXPECT_TRUE(resultMesh.groups[19].elements[0].isQuad());
    EXPECT_TRUE(resultMesh.groups[19].elements[1].isLine());


    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedGroup = expectedElements[g];


        for (std::size_t e = 0; e < expectedGroup.size(); ++e) {
            auto& resultElement = resultGroup.elements[e];
            auto& expectedElement = expectedGroup[e];

            for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
            }
        }
    }
}

}
