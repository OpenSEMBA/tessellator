#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

#include "MeshFixtures.h"

#include "Structurer.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace utils;
using namespace meshFixtures;

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

    // *---------*      2--------*
    // |  2      |      ‖         |
	// |  |      |      ‖         |
    // |  |  _-1 |  ->  ‖         |
	// |  0-‾    |      ‖         |
    // *---------*      0========1

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

    Mesh& resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    ASSERT_EQ(resultMesh.groups[0].elements.size(), 1);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 1);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 1);

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
        }
    }
    

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedElement = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        for (std::size_t i = 0; i < expectedElement.vertices.size(); ++i) {
            EXPECT_EQ(resultGroup.elements[0].vertices[i], expectedElement.vertices[i]);
        }
    }
}

TEST_F(StructurerTest, transformSingleSegmentsIntoTwoStructuredElements)
{

    // *-----------*  {0.5->1}======={1->2}    *-----------*       *----------{3->2}
    // |      1    |      ‖            |       |        3   |      |             ‖
    // |     /     |      ‖            |       |       /    |      |             ‖
    // |    /      |  ->  ‖            |       |      /     |  ->  |             ‖
    // |  0        |      ‖            |       |    2       |      |             ‖
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

    Mesh& resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    ASSERT_EQ(resultMesh.groups[0].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[3].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[4].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[5].elements.size(), 2);

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

        for (std::size_t i = 0; i < expectedGroup.size(); ++i) {
            auto& resultElement = resultGroup.elements[i];
            auto& expectedElement = expectedGroup[i];

            for (std::size_t j = 0; j < expectedElement.vertices.size(); ++j) {
                EXPECT_EQ(resultElement.vertices[j], expectedElement.vertices[j]);
            }
        }
    }
}



TEST_F(StructurerTest, transformSingleSegmentsWithinDiagonalIntoTwoStructuredElements)
{

    // y                y                       y               y
    // *-------*        *-------{1->2}          *------*    {3.5->4}======{4->5}
    // |      1|        |          ‖            |      4|        ‖            |
    // |    ╱  |        |          ‖            |    ╱  |        ‖            |
    // |  ╱    |   ->   |          ‖            |  ╱    |   ->   ‖            |
    // |0      |        |          ‖            |3      |        ‖            |
    // *-------* x      0======{0.5->1} x       *------* z    {3->0}---------* z

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({ 0.1, 0.1, 0.0 }),    // 0 First Segment, First Point, X-Y Plane
        Coordinate({ 0.9, 0.9, 0.0 }),    // 1 First Segment, Final Point, X-Y Plane
        Coordinate({ 0.1, 0.0, 0.1 }),    // 2 Second Segment, First Point, X-Z Plane
        Coordinate({ 0.9, 0.0, 0.9 }),    // 3 Second Segment, Final Point, X-Z Plane
        Coordinate({ 0.0, 0.1, 0.1 }),    // 4 Third Segment, First Point, Y-Z Plane
        Coordinate({ 0.0, 0.9, 0.9 }),    // 5 Third Segment, Final Point, X-Z Plane
    };
    mesh.groups.resize(3);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({2, 3}, Element::Type::Line)
    };
    mesh.groups[2].elements = {
        Element({4, 5}, Element::Type::Line)
    };

    Coordinates expectedCoordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 All Segments, First Point, X-Y and X-Z Planes
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First and Second Segments, Middle Point, X-Y and X-Z Planes
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First Segment, Final Point, X-Y Plane
        Coordinate({ 1.0, 0.0, 1.0 }),    // 3 Second Segment, Final Point, X-Z Plane
        Coordinate({ 0.0, 1.0, 0.0 }),    // 4 Third Segment, Middle Point, X-Z Plane
        Coordinate({ 0.0, 1.0, 1.0 }),    // 5 Third Segment, Final Point, X-Z Plane
    };

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
        }
    };

    Mesh& resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    ASSERT_EQ(resultMesh.groups[0].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 2);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 2);
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

        for (std::size_t i = 0; i < expectedGroup.size(); ++i) {
            auto& resultElement = resultGroup.elements[i];
            auto& expectedElement = expectedGroup[i];

            for (std::size_t j = 0; j < expectedElement.vertices.size(); ++j) {
                EXPECT_EQ(resultElement.vertices[j], expectedElement.vertices[j]);
            }
        }
    }
}


TEST_F(StructurerTest, transformSingleSegmentsIntoThreeStructuredElements)
{

    //     *-------------*               *----------{1->3}  
    //    /|            /|              /|            /‖    
    //   / |           / |             / |           / ‖    
    // z/  |        1 /  |           z/  |          /  ‖    
    // *---┼-----==‾+*   |    ->     *---┼---------*   ‖    
    // |   |y _-‾   ¦|   |           |   |y        |   ‖    
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
        Coordinate({ 0.0,  0.02, 0.04 }),    // 0 First, Third and Fourth Segments, First Point
        Coordinate({ 1.0,  0.98, 0.52 }),    // 1 First and Second Segments, Final Point
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
        Coordinate({ 0.0, 1.0, 1.0 }),    // 5 Second and Third Segments, Third Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 6 Fourth Segment, Second Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 7 Fourth Segment, Third Point
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
            Element({1, 5}, Element::Type::Line),
            Element({5, 3}, Element::Type::Line),
        },
        {
            Element({0, 6}, Element::Type::Line),
            Element({6, 7}, Element::Type::Line),
            Element({7, 3}, Element::Type::Line),
        },
    };

    Mesh& resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    ASSERT_EQ(resultMesh.groups[0].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[3].elements.size(), 3);
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

        for (std::size_t i = 0; i < expectedGroup.size(); ++i)
        {
            auto& resultElement = resultGroup.elements[i];
            auto& expectedElement = expectedGroup[i];

            for (std::size_t j = 0; j < expectedElement.vertices.size(); ++j) {
                EXPECT_EQ(resultElement.vertices[j], expectedElement.vertices[j]);
            }
        }
    }
}


TEST_F(StructurerTest, transformSingleSegmentsWithinDiagonalIntoThreeStructuredElements)
{

    //     *-------------*               *----------{1->3}  
    //    /|           2/|              /|            /‖    
    //   / |          ╱¦ |             / |           / ‖    
    // z/  |        ╱ /¦ |           z/  |          /  ‖    
    // *---┼------╱--* ¦ |    ->     *---┼---------*   ‖    
    // |   |y   ╱    | ¦ |           |   |y        |   ‖    
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
        Coordinate({ 0.0, 0.0, 0.0 }),    // 0 First Segment, First Point; Second Segment, Second Point; Fifth Segment, Third Point and Eight Segment, Final Point
        Coordinate({ 1.0, 0.0, 0.0 }),    // 1 First Segment, Second Point; Second Segment, First Point; Third Segment, Third Point and Seventh Segment, Final point
        Coordinate({ 1.0, 1.0, 0.0 }),    // 2 First Segment, Third Point; Third Segment, Second Point; Fourth Segment, Final Point and Fifth Segment, First Point
        Coordinate({ 1.0, 1.0, 1.0 }),    // 3 First Segment, Final Point; Fourth Segment, Third Point; Seventh Segment, Second Point and Eight Segment, First Point
        Coordinate({ 0.0, 1.0, 0.0 }),    // 4 Second Segment, Third Point; Third Segment, First Point; Fifth Segment, Second Point and Sixth Segment, Final Point
        Coordinate({ 0.0, 1.0, 1.0 }),    // 5 Second Segment, Final Point; Sixth Segment, Third Point; Seventh Segment, First Point and Eight Segment, Second Point
        Coordinate({ 1.0, 0.0, 1.0 }),    // 6 Third Segment, Final Point; Fourth Segment, Second Point; Sixth Segment, First Point and Seventh Segment, Third Point
        Coordinate({ 0.0, 0.0, 1.0 }),    // 7 Fourth Segment, First Point and Fifth Segment, Final Point; Sixth Segment, Second Point and Eight Segment, Third Point
    };

    std::vector<Elements> expectedElements = {
        {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
            Element({2, 3}, Element::Type::Line),
        },
        {
            Element({1, 0}, Element::Type::Line),
            Element({0, 4}, Element::Type::Line),
            Element({4, 5}, Element::Type::Line),
        },
        {
            Element({4, 2}, Element::Type::Line),
            Element({2, 1}, Element::Type::Line),
            Element({1, 6}, Element::Type::Line),
        },
        {
            Element({7, 6}, Element::Type::Line),
            Element({6, 3}, Element::Type::Line),
            Element({3, 2}, Element::Type::Line),
        },
        {
            Element({2, 4}, Element::Type::Line),
            Element({4, 0}, Element::Type::Line),
            Element({0, 7}, Element::Type::Line),
        },
        {
            Element({6, 7}, Element::Type::Line),
            Element({7, 5}, Element::Type::Line),
            Element({5, 4}, Element::Type::Line),
        },
        {
            Element({5, 3}, Element::Type::Line),
            Element({3, 6}, Element::Type::Line),
            Element({6, 1}, Element::Type::Line),
        },
        {
            Element({3, 5}, Element::Type::Line),
            Element({5, 7}, Element::Type::Line),
            Element({7, 0}, Element::Type::Line),
        },
    };

    Mesh& resultMesh = Structurer{ mesh }.getMesh();

    ASSERT_EQ(resultMesh.coordinates.size(), expectedCoordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedElements.size());
    ASSERT_EQ(resultMesh.groups[0].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[1].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[2].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[3].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[4].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[5].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[6].elements.size(), 3);
    ASSERT_EQ(resultMesh.groups[7].elements.size(), 3);
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

        for (std::size_t i = 0; i < expectedGroup.size(); ++i) {
            auto& resultElement = resultGroup.elements[i];
            auto& expectedElement = expectedGroup[i];

            for (std::size_t j = 0; j < expectedElement.vertices.size(); ++j) {
                EXPECT_EQ(resultElement.vertices[j], expectedElement.vertices[j]);
            }
        }
    }
}