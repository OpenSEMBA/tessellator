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

    auto grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    Structurer structurer{ grid };
    ASSERT_NO_THROW(structurer.grid());

    for (CellDir i = 0; i < grid[X].size(); ++i) {
        CoordinateDir xDirection;
        if (i != grid[X].size() - 1)
            xDirection = lowerCoordinateValue + step * i;
        else
            xDirection = upperCoordinateValue;

        for (CellDir j = 0; j < grid[Y].size(); ++j) {
            CoordinateDir yDirection;
            if (j != grid[Y].size() - 1)
                yDirection = lowerCoordinateValue + step * j;
            else
                yDirection = upperCoordinateValue;

            for (CellDir k = 0; k < grid[Z].size(); ++k) {
                CoordinateDir zDirection;
                if (k != grid[Z].size() - 1)
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

    auto grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    Structurer structurer{ grid };
    ASSERT_NO_THROW(structurer.grid());

    for (CellDir i = 0; i < grid[X].size() - 1; ++i) {
        for (CellDir j = 0; j < grid[Y].size() - 1; ++j) {
            for (CellDir k = 0; k < grid[Z].size() - 1; ++k) {
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
