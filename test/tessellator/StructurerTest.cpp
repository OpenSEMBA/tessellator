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
        CoordinateDir xDirection;
        if (i != mesh.grid[X].size() - 1)
            xDirection = lowerCoordinateValue + step * i;
        else
            xDirection = upperCoordinateValue;

        for (CellDir j = 0; j < mesh.grid[Y].size(); ++j) {
            CoordinateDir yDirection;
            if (j != mesh.grid[Y].size() - 1)
                yDirection = lowerCoordinateValue + step * j;
            else
                yDirection = upperCoordinateValue;

            for (CellDir k = 0; k < mesh.grid[Z].size(); ++k) {
                CoordinateDir zDirection;
                if (k != mesh.grid[Z].size() - 1)
                    zDirection = lowerCoordinateValue + step * k;
                else
                    zDirection = upperCoordinateValue;
                Coordinate coordinate({ xDirection, yDirection, zDirection });
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

    float quarterStep = step / 4.0f;
    float halfStep = step / 2.0f;
    float threeQuarterStep = step * 3.0f / 4.0f;

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
                for (std::size_t axis = 0; axis < 3; ++axis)
                    coordinateBase[axis] = lowerCoordinateValue + step * expectedLowerCell[axis];

                Coordinate newCoordinate;
                for (std::size_t axis = 0; axis < 3; ++axis)
                    newCoordinate[axis] = coordinateBase[axis] + quarterStep;

                auto resultCell = structurer.calculateStructuredCell(newCoordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedLowerCell[axis]);
                }

                for (std::size_t axis = 0; axis < 3; ++axis)
                    newCoordinate[axis] = coordinateBase[axis] + halfStep;

                resultCell = structurer.calculateStructuredCell(newCoordinate);

                for (std::size_t axis = 0; axis < 3; ++axis) {
                    EXPECT_EQ(resultCell[axis], expectedUpperCell[axis]);
                }

                for (std::size_t axis = 0; axis < 3; ++axis)
                    newCoordinate[axis] = coordinateBase[axis] + threeQuarterStep;

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

    // * - - - - *      2 - - - - *
    // |  2      |      ‖         |
	// |  |      |      ‖         |
    // |  |  / 1 |  ->  ‖         |
	// |  0 /    |      ‖         |
    // * - - - - *      0 = = = = 1

    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Coordinate({-3.5, -3.5, -5.0 }),
        Coordinate({-1.5, -3.0, -5.0 }),
        Coordinate({-3.5, -1.5, -5.0 }),
        Coordinate({-3.5, -5.0, -1.5 }),
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
        Coordinate({0.0, 0.0, 0.0 }),
        Coordinate({1.0, 0.0, 0.0 }),
        Coordinate({0.0, 1.0, 0.0 }),
        Coordinate({0.0, 0.0, 1.0 }),
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

    for (std::size_t i = 0; i < expectedCoordinates.size(); ++i) 
        for (std::size_t axis = 0; axis < 3; ++axis)
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedCoordinates[i][axis]);
    

    for (std::size_t g = 0; g < expectedElements.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedElement = expectedElements[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        for (std::size_t i = 0; i < expectedElement.vertices.size(); ++i)
            EXPECT_EQ(resultGroup.elements[0].vertices[i], expectedElement.vertices[i]);
    }
}