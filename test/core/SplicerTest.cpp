#include "gtest/gtest.h"
#include "utils/Geometry.h"
#include "utils/GridTools.h"
#include "core/Splicer.h"

namespace meshlib::core {

using namespace utils;

class SplicerTest : public ::testing::Test {
public:
protected:
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

    static void assertElementsListEquals(const Elements& expectedElements, const Elements& resultElements) {
        ASSERT_EQ(expectedElements.size(), resultElements.size());

        for (ElementId e = 0; e < expectedElements.size(); ++e) {
            const Element& expectedElement = expectedElements[e];
            const Element& resultElement = resultElements[e];

            EXPECT_EQ(resultElement.vertices.size(), expectedElement.vertices.size())
                << "Current Element: #" << e << std::endl;
            EXPECT_EQ(resultElement.type, expectedElement.type)
                << "Current Element: #" << e << std::endl;

            for (std::size_t v = 0; v < resultElement.vertices.size(); ++v) {
                EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                    << "Current Element: #" << e << std::endl
                    << "Current Vertex: #" << v << std::endl;
            }
        }
    }
};

TEST_F(SplicerTest, spliceTriangleWithPointsInFirstSide)
{
    //                                                       
    //                .                                0.
    //               /0\                               /|\
    //              /   \                             /⎹ ⎸\
    //             /     \                           / | | \
    //            /       \                         /  ⎸ ⎹  \
    //           /         \                       /  ⎹   ⎸  \
    //          /           \         -->         /   |   |   \
    //         /             \                   /    ⎸   ⎹    \
    //        /               \                 /    ⎹     ⎸    \
    //       /_________________\               /_____|_____|_____\
    //      1\   3/\    4/\    /2             1\   3/\    4/\    /2
    //        \  /  \   /  \  /                 \  /  \   /  \  /
    //         \/____\ /____\/                   \/____\ /____\/
    //         5      6     7                    5      6     7

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 4.00, 4.00, 0.00 }),   // 3
        Coordinate({ 6.00, 4.00, 0.00 }),   // 4
        Coordinate({ 3.00, 3.00, 0.00 }),   // 5
        Coordinate({ 5.00, 3.00, 0.00 }),   // 6
        Coordinate({ 7.00, 3.00, 0.00 }),   // 7
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 1, 5, 3 }, Element::Type::Surface),  // 1
        Element({ 1, 5, 3 }, Element::Type::Surface),  // 2
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 3
        Element({ 2, 4, 7 }, Element::Type::Surface),  // 4
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 0, 1, 3 }, Element::Type::Surface),  // 0
        Element({ 0, 3, 4 }, Element::Type::Surface),  // 1
        Element({ 0, 4, 2 }, Element::Type::Surface),  // 2

        Element({ 1, 5, 3 }, Element::Type::Surface),  // 3
        Element({ 1, 5, 3 }, Element::Type::Surface),  // 4
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 5
        Element({ 2, 4, 7 }, Element::Type::Surface),  // 6
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithPointsInSecondSide)
{
    //
    //                    ._____7                        ._____7       
    //                   /0\    /\                      /0\    /\      
    //                  /   \  /  \                    /   \  /  \     
    //                 /     \/____\6                 /    4\/____\6   
    //                /      4\    /\                /     ⎽⎼⎻⎺\    /\   
    //               /         \  /  \              /    ⎽⎼⎻⎺   \  /  \  
    //              /           \/____\   -->      /   ⎽⎼⎻⎺     3\/____\ 
    //             /            3\    /5          /  ⎽⎼⎻⎺    __-‾‾\    /5
    //            /               \  /           / ⎽⎼⎻⎺___-‾‾      \  /  
    //           /_________________\/           /__-______________\/   
    //          1                   2          1                   2 
    //              

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 7.00, 5.00, 0.00 }),   // 3
        Coordinate({ 6.00, 6.00, 0.00 }),   // 4
        Coordinate({ 9.00, 5.00, 0.00 }),   // 5
        Coordinate({ 8.00, 6.00, 0.00 }),   // 6
        Coordinate({ 7.00, 7.00, 0.00 }),   // 7
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 2, 5, 3 }, Element::Type::Surface),  // 1
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 2
        Element({ 0, 4, 7 }, Element::Type::Surface),  // 3
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 1, 2, 3 }, Element::Type::Surface),  // 0
        Element({ 1, 3, 4 }, Element::Type::Surface),  // 1
        Element({ 1, 4, 0 }, Element::Type::Surface),  // 2

        Element({ 2, 5, 3 }, Element::Type::Surface),  // 3
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 4
        Element({ 0, 4, 7 }, Element::Type::Surface),  // 5
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithPointsInThirdSide)
{
    //
    //             5______.                              5______.
    //             /\    /0\                             /\    /0\
    //            /  \  /   \                           /  \  /   \
    //          6/____\/     \                        6/____\/3    \
    //          /\    /3      \                       /\    / ⎺⎻⎼⎽    \
    //         /  \  /         \                     /  \  /    ⎺⎻⎼⎽   \
    //       7/____\/           \          ->      7/____\/4      ⎺⎻⎼⎽  \
    //        \    /4            \                  \    /‾‾--__    ⎺⎻⎼⎽ \
    //         \  /               \                  \  /       ‾‾-___⎺⎻⎼⎽\
    //          \/_________________\                  \/______________-__\
    //          1                   2                 1                   2
    //              

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 4.00, 6.00, 0.00 }),   // 3
        Coordinate({ 3.00, 5.00, 0.00 }),   // 4
        Coordinate({ 3.00, 7.00, 0.00 }),   // 5
        Coordinate({ 2.00, 6.00, 0.00 }),   // 6
        Coordinate({ 1.00, 5.00, 0.00 }),   // 7
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 0, 5, 3 }, Element::Type::Surface),  // 1
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 2
        Element({ 1, 4, 7 }, Element::Type::Surface),  // 3
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 2, 0, 3 }, Element::Type::Surface),  // 0
        Element({ 2, 3, 4 }, Element::Type::Surface),  // 1
        Element({ 2, 4, 1 }, Element::Type::Surface),  // 2

        Element({ 0, 5, 3 }, Element::Type::Surface),  // 3
        Element({ 3, 6, 4 }, Element::Type::Surface),  // 4
        Element({ 1, 4, 7 }, Element::Type::Surface),  // 5
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithPointsInFirstAndSecondSides)
{
    //
    //                ._____12                         0.______12
    //               /0\    /\                         /|\    /\
    //              /   \  /  \                       /⎹ ⎸\  /  \
    //             /     \/____\11                   / | |9\/____\11
    //            /      9\    /\                   /  ⎸ ⎹ |\    /\
    //           /         \  /  \                 /  ⎹   ⎸| \  /  \
    //          /           \/____\     -->       /   |   ||  \/____\
    //         /            8\    /10            /    ⎸   ⎹| 8/\    /10
    //        /               \  /              /    ⎹     ⎸ /  \  /
    //       /_________________\/              /_____|_____|/____\/
    //      1\   3/\    4/\    /2             1\   3/\    4/\    /2
    //        \  /  \   /  \  /                 \  /  \   /  \  /
    //         \/____\ /____\/                   \/____\ /____\/
    //         5      6     7                    5      6     7

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 4.00, 4.00, 0.00 }),   // 3
        Coordinate({ 6.00, 4.00, 0.00 }),   // 4
        Coordinate({ 3.00, 3.00, 0.00 }),   // 5
        Coordinate({ 5.00, 3.00, 0.00 }),   // 6
        Coordinate({ 7.00, 3.00, 0.00 }),   // 7
        Coordinate({ 7.00, 5.00, 0.00 }),   // 8
        Coordinate({ 6.00, 6.00, 0.00 }),   // 9
        Coordinate({ 9.00, 5.00, 0.00 }),   // 10
        Coordinate({ 8.00, 6.00, 0.00 }),   // 11
        Coordinate({ 7.00, 7.00, 0.00 }),   // 12
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({   0,  1,  2  }, Element::Type::Surface),  // 0
        Element({   1,  5,  3  }, Element::Type::Surface),  // 1
        Element({   1,  5,  3  }, Element::Type::Surface),  // 2
        Element({   3,  6,  4  }, Element::Type::Surface),  // 3
        Element({   2,  4,  7  }, Element::Type::Surface),  // 4
        Element({   2, 10,  8  }, Element::Type::Surface),  // 5
        Element({   8, 11,  9  }, Element::Type::Surface),  // 6
        Element({   0,  9, 12  }, Element::Type::Surface),  // 7
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({   0,  1,  3  }, Element::Type::Surface),  // 0
        Element({   0,  3,  4  }, Element::Type::Surface),  // 1

        Element({   4,  2,  8  }, Element::Type::Surface),  // 2
        Element({   4,  8,  9  }, Element::Type::Surface),  // 3
        Element({   4,  9,  0  }, Element::Type::Surface),  // 4


        Element({   1,  5,  3  }, Element::Type::Surface),  // 5
        Element({   1,  5,  3  }, Element::Type::Surface),  // 6
        Element({   3,  6,  4  }, Element::Type::Surface),  // 7
        Element({   2,  4,  7  }, Element::Type::Surface),  // 8
        Element({   2, 10,  8  }, Element::Type::Surface),  // 9
        Element({   8, 11,  9  }, Element::Type::Surface),  // 10
        Element({   0,  9, 12  }, Element::Type::Surface),  // 11
    };
}

TEST_F(SplicerTest, spliceTriangleWithPointsInSecondAndThirdSides)
{
    //
    //            10______._____7                   10______.______7
    //             /\    /0\    /\                   /\    /0\    /\
    //            /  \  /   \  /  \                 /  \  /   \  /  \
    //         11/____\/     \/____\6            11/____\/_____\/____\6
    //          /\    /8     4\    /\             /\    /8  _-⎽⎼⎻⎺\4   /\
    //         /  \  /         \  /  \           /  \  /  _-⎽⎼⎻⎺   \  /  \
    //      12/____\/           \/____\   --> 12/____\/_-‾⎽⎼⎻⎺     3\/____\
    //        \    /9           3\    /5        \    /9 ⎽⎼⎻⎺    __-‾‾\    /5
    //         \  /               \  /           \  / ⎽⎼⎻⎺___-‾‾      \  /
    //          \/_________________\/             \/__-______________\/
    //          1                   2             1                   2

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 7.00, 5.00, 0.00 }),   // 3
        Coordinate({ 6.00, 6.00, 0.00 }),   // 4
        Coordinate({ 9.00, 5.00, 0.00 }),   // 5
        Coordinate({ 8.00, 6.00, 0.00 }),   // 6
        Coordinate({ 7.00, 7.00, 0.00 }),   // 7
        Coordinate({ 4.00, 6.00, 0.00 }),   // 8
        Coordinate({ 3.00, 5.00, 0.00 }),   // 9
        Coordinate({ 3.00, 7.00, 0.00 }),   // 10
        Coordinate({ 2.00, 6.00, 0.00 }),   // 11
        Coordinate({ 1.00, 5.00, 0.00 }),   // 12
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({   0,  1,  2  }, Element::Type::Surface),  // 0
        Element({   2,  5,  3  }, Element::Type::Surface),  // 1
        Element({   3,  6,  4  }, Element::Type::Surface),  // 2
        Element({   0,  4,  7  }, Element::Type::Surface),  // 3
        Element({   0, 10,  8  }, Element::Type::Surface),  // 4
        Element({   8, 11,  9  }, Element::Type::Surface),  // 5
        Element({   1,  9, 12  }, Element::Type::Surface),  // 6
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({   1,  2,  3  }, Element::Type::Surface),  // 0
        Element({   1,  3,  4  }, Element::Type::Surface),  // 1

        Element({   4,  0,  8  }, Element::Type::Surface),  // 2
        Element({   4,  8,  9  }, Element::Type::Surface),  // 3
        Element({   4,  9,  1  }, Element::Type::Surface),  // 4


        Element({   2,  5,  3  }, Element::Type::Surface),  // 5
        Element({   3,  6,  4  }, Element::Type::Surface),  // 6
        Element({   0,  4,  7  }, Element::Type::Surface),  // 7
        Element({   0, 10,  8  }, Element::Type::Surface),  // 8
        Element({   8, 11,  9  }, Element::Type::Surface),  // 9
        Element({   1,  9, 12  }, Element::Type::Surface),  // 10
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithPointsInFirstAndThirdSides)
{
    //
    //            10______.                           10______.0
    //             /\    /0\                           /\    /|\
    //            /  \  /   \                         /  \  /⎹ ⎸\
    //         11/____\/     \                     11/____\/ | | \
    //          /\    /8      \                     /\   8/| ⎸ ⎹  \
    //         /  \  /         \                   /  \  / |⎹   ⎸  \
    //      12/____\/           \         -->   12/____\/  ||   |   \
    //        \    /9            \                \   9/\  |⎸   ⎹    \    
    //         \  /               \                \  /  \ ⎹     ⎸    \
    //          \/_________________\                \/____\|_____|_____\
    //          1\   3/\    4/\    /2               1\   3/\    4/\    /2
    //            \  /  \   /  \  /                   \  /  \   /  \  /
    //             \/____\ /____\/                     \/____\ /____\/
    //             5      6     7                      5      6     7

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 4.00, 4.00, 0.00 }),   // 3
        Coordinate({ 6.00, 4.00, 0.00 }),   // 4
        Coordinate({ 3.00, 3.00, 0.00 }),   // 5
        Coordinate({ 5.00, 3.00, 0.00 }),   // 6
        Coordinate({ 7.00, 3.00, 0.00 }),   // 7
        Coordinate({ 4.00, 6.00, 0.00 }),   // 8 
        Coordinate({ 3.00, 5.00, 0.00 }),   // 9 
        Coordinate({ 3.00, 7.00, 0.00 }),   // 10
        Coordinate({ 2.00, 6.00, 0.00 }),   // 11
        Coordinate({ 1.00, 5.00, 0.00 }),   // 12
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({   0,  1,  2  }, Element::Type::Surface),  // 0
        Element({   1,  5,  3  }, Element::Type::Surface),  // 1
        Element({   1,  5,  3  }, Element::Type::Surface),  // 2
        Element({   3,  6,  4  }, Element::Type::Surface),  // 3
        Element({   2,  4,  7  }, Element::Type::Surface),  // 4
        Element({   0, 10,  8  }, Element::Type::Surface),  // 5
        Element({   8, 11,  9  }, Element::Type::Surface),  // 6
        Element({   1,  9, 12  }, Element::Type::Surface),  // 7
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({   0,  3,  4  }, Element::Type::Surface),  // 0
        Element({   0,  4,  2  }, Element::Type::Surface),  // 1

        Element({   3,  0,  8  }, Element::Type::Surface),  // 2
        Element({   3,  8,  9  }, Element::Type::Surface),  // 3
        Element({   3,  9,  1  }, Element::Type::Surface),  // 4


        Element({   1,  5,  3  }, Element::Type::Surface),  // 5
        Element({   1,  5,  3  }, Element::Type::Surface),  // 6
        Element({   3,  6,  4  }, Element::Type::Surface),  // 7
        Element({   2,  4,  7  }, Element::Type::Surface),  // 8
        Element({   0, 10,  8  }, Element::Type::Surface),  // 9
        Element({   8, 11,  9  }, Element::Type::Surface),  // 10
        Element({   1,  9, 12  }, Element::Type::Surface),  // 11
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithPointsInAllSides)
{
    //
    //            15______._____12                    ______.______
    //             /\    /0\    /\                   /\    /|\    /\
    //            /  \  /   \  /  \                 /  \  /⎹ ⎸\  /  \
    //         16/____\/     \/____\11             /____\/ | | \/____\
    //          /\    /13    9\    /\             /\    /| ⎸ ⎹ |\    /\
    //         /  \  /         \  /  \           /  \  / |⎹   ⎸| \  /  \
    //      17/____\/           \/____\   -->   /____\/  ||   ||  \/____\
    //        \    /14          8\    /10       \    /\  |⎸   ⎹|  /\    /
    //         \  /               \  /           \  /  \ ⎹     ⎸ /  \  /
    //          \/_________________\/             \/____\|_____|/____\/
    //          1\   3/\    4/\    /2              \    /\     /\    /
    //            \  /  \   /  \  /                 \  /  \   /  \  /
    //             \/____\ /____\/                   \/____\ /____\/
    //             5      6     7

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 4.00, 4.00, 0.00 }),   // 3
        Coordinate({ 6.00, 4.00, 0.00 }),   // 4
        Coordinate({ 3.00, 3.00, 0.00 }),   // 5
        Coordinate({ 5.00, 3.00, 0.00 }),   // 6
        Coordinate({ 7.00, 3.00, 0.00 }),   // 7
        Coordinate({ 7.00, 5.00, 0.00 }),   // 8
        Coordinate({ 6.00, 6.00, 0.00 }),   // 9
        Coordinate({ 9.00, 5.00, 0.00 }),   // 10
        Coordinate({ 8.00, 6.00, 0.00 }),   // 11
        Coordinate({ 7.00, 7.00, 0.00 }),   // 12
        Coordinate({ 4.00, 6.00, 0.00 }),   // 13
        Coordinate({ 3.00, 5.00, 0.00 }),   // 14
        Coordinate({ 3.00, 7.00, 0.00 }),   // 15
        Coordinate({ 2.00, 6.00, 0.00 }),   // 16
        Coordinate({ 1.00, 5.00, 0.00 }),   // 17
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({   0,  1,  2  }, Element::Type::Surface),  // 0
        Element({   1,  5,  3  }, Element::Type::Surface),  // 1
        Element({   1,  5,  3  }, Element::Type::Surface),  // 2
        Element({   3,  6,  4  }, Element::Type::Surface),  // 3
        Element({   2,  4,  7  }, Element::Type::Surface),  // 4
        Element({   2, 10,  8  }, Element::Type::Surface),  // 5
        Element({   8, 11,  9  }, Element::Type::Surface),  // 6
        Element({   0,  9, 12  }, Element::Type::Surface),  // 7
        Element({   0, 15, 13  }, Element::Type::Surface),  // 8
        Element({  13, 16, 14  }, Element::Type::Surface),  // 9
        Element({   1, 14, 17  }, Element::Type::Surface),  // 10
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({   0,  3,  4  }, Element::Type::Surface),  // 0

        Element({   4,  2,  8  }, Element::Type::Surface),  // 1
        Element({   4,  8,  9  }, Element::Type::Surface),  // 2
        Element({   4,  9,  0  }, Element::Type::Surface),  // 3


        Element({   3,  0, 13  }, Element::Type::Surface),  // 4
        Element({   3, 13, 14  }, Element::Type::Surface),  // 5
        Element({   3, 14,  1  }, Element::Type::Surface),  // 6


        Element({   1,  5,  3  }, Element::Type::Surface),  // 6
        Element({   1,  5,  3  }, Element::Type::Surface),  // 7
        Element({   3,  6,  4  }, Element::Type::Surface),  // 8
        Element({   2,  4,  7  }, Element::Type::Surface),  // 9
        Element({   2, 10,  8  }, Element::Type::Surface),  // 10
        Element({   8, 11,  9  }, Element::Type::Surface),  // 11
        Element({   0,  9, 12  }, Element::Type::Surface),  // 12
        Element({   0, 15, 13  }, Element::Type::Surface),  // 13
        Element({  13, 16, 14  }, Element::Type::Surface),  // 14
        Element({   1, 14, 17  }, Element::Type::Surface),  // 15
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithOnePointInFirstAndSecondSides)
{
    //
    //                    ._____.8                          0_____.8
    //                   /0\    ⎸                          /|\    ⎸
    //                  /   \   ⎸                         / | \   ⎸
    //                 /     \  ⎸                        /  |  \  ⎸
    //                /       \ ⎸                       /   |   \ ⎸
    //               /        6\----.7                 /    |   6\----.7
    //              /           \   |     -->         /     |   / \   |
    //             /             \  |                /      |  /   \  |
    //            /               \ |               /       | /     \ |
    //           /_________________\|              /________|/_______\|
    //          1 ⟍     3/\      ⟋ 2              1 ⟍     3/\      ⟋ 2
    //              ⟍   /  \   ⟋                      ⟍   /  \   ⟋
    //                ⟍/____\⟋                          ⟍/____\⟋
    //                 4     5                           4     5

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 5.00, 4.00, 0.00 }),   // 3
        Coordinate({ 4.00, 3.00, 0.00 }),   // 4
        Coordinate({ 6.00, 3.00, 0.00 }),   // 5
        Coordinate({ 6.50, 5.50, 0.00 }),   // 6
        Coordinate({ 8.00, 5.50, 0.00 }),   // 7
        Coordinate({ 6.50, 7.00, 0.00 }),   // 8
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 3, 4, 5 }, Element::Type::Surface),  // 1
        Element({ 2, 7, 6 }, Element::Type::Surface),  // 2
        Element({ 0, 6, 8 }, Element::Type::Surface),  // 3
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 0, 1, 3 }, Element::Type::Surface),  // 0

        Element({ 3, 2, 6 }, Element::Type::Surface),  // 1
        Element({ 3, 6, 0 }, Element::Type::Surface),  // 2

        Element({ 3, 4, 5 }, Element::Type::Surface),  // 3
        Element({ 2, 7, 6 }, Element::Type::Surface),  // 4
        Element({ 0, 6, 8 }, Element::Type::Surface),  // 5
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithOnePointInSecondAndThirdSides)
{
    //
    //             7._____._____.5                   7._____0_____.5
    //              ⎹    /0\    ⎸                     ⎹    / \    ⎸
    //              ⎹   /   \   ⎸                     ⎹   /   \   ⎸
    //              ⎹  /     \  ⎸                     ⎹  /     \  ⎸
    //              ⎹ /       \ ⎸                     ⎹6/       \3⎸
    //         8.----/6       3\----.4           8.----/---------\----.4
    //          |   /           \   |     -->     |   /       _-‾ \   |
    //          |  /             \  |             |  /    _-‾‾     \  |
    //          | /               \ |             | /__-‾‾          \ |
    //          |/_________________\|             |/=________________\|
    //          1                  2              1                   2


    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 6.50, 5.50, 0.00 }),   // 3
        Coordinate({ 8.00, 5.50, 0.00 }),   // 4
        Coordinate({ 6.50, 7.00, 0.00 }),   // 5
        Coordinate({ 3.50, 5.50, 0.00 }),   // 6
        Coordinate({ 3.50, 7.00, 0.00 }),   // 7 
        Coordinate({ 2.00, 5.50, 0.00 }),   // 8 
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 2, 4, 3 }, Element::Type::Surface),  // 1
        Element({ 0, 3, 5 }, Element::Type::Surface),  // 2
        Element({ 0, 7, 6 }, Element::Type::Surface),  // 3
        Element({ 1, 6, 8 }, Element::Type::Surface),  // 4
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 1, 2, 3 }, Element::Type::Surface),  // 0

        Element({ 3, 0, 6 }, Element::Type::Surface),  // 1
        Element({ 3, 6, 1 }, Element::Type::Surface),  // 2

        Element({ 2, 4, 3 }, Element::Type::Surface),  // 3
        Element({ 0, 3, 5 }, Element::Type::Surface),  // 4
        Element({ 0, 7, 6 }, Element::Type::Surface),  // 5
        Element({ 1, 6, 8 }, Element::Type::Surface),  // 6
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithOnePointInFirstAndThirdSides)
{
    //
    //             7._____.                          7._____0
    //              ⎹    /0\                          ⎹    /|\
    //              ⎹   /   \                         ⎹   / | \
    //              ⎹  /     \                        ⎹  /  |  \
    //              ⎹ /       \                       ⎹ /   |   \
    //        8 .----/6        \                8 .----/6   |    \
    //          |   /           \         -->     |   / \   |     \
    //          |  /             \                |  /   \  |      \
    //          | /               \               | /     \ |       \
    //          |/_________________\              |/_______\|________\
    //          1 ⟍     3/\      ⟋ 2              1 ⟍     3/\      ⟋ 2
    //              ⟍   /  \   ⟋                      ⟍   /  \   ⟋       
    //                ⟍/____\⟋                          ⟍/____\⟋
    //                 4     5                           4     5

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 5.00, 4.00, 0.00 }),   // 3
        Coordinate({ 4.00, 3.00, 0.00 }),   // 4
        Coordinate({ 6.00, 3.00, 0.00 }),   // 5
        Coordinate({ 3.50, 5.50, 0.00 }),   // 6
        Coordinate({ 3.50, 7.00, 0.00 }),   // 7 
        Coordinate({ 2.00, 5.50, 0.00 }),   // 8 
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
        Element({ 3, 4, 5 }, Element::Type::Surface),  // 1
        Element({ 0, 7, 6 }, Element::Type::Surface),  // 2
        Element({ 1, 6, 8 }, Element::Type::Surface),  // 3
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 0, 3, 2 }, Element::Type::Surface),  // 0

        Element({ 3, 0, 6 }, Element::Type::Surface),  // 1
        Element({ 3, 6, 1 }, Element::Type::Surface),  // 2

        Element({ 3, 4, 5 }, Element::Type::Surface),  // 3
        Element({ 0, 7, 6 }, Element::Type::Surface),  // 4
        Element({ 1, 6, 8 }, Element::Type::Surface),  // 5
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, spliceTriangleWithOnePointInAllSides)
{
    //
    //            10._____._____.8                  10._____0_____.8
    //              ⎹    /0\    ⎸                     ⎹    /|\    ⎸
    //              ⎹   /   \   ⎸                     ⎹   / | \   ⎸
    //              ⎹  /     \  ⎸                     ⎹  /  |  \  ⎸
    //              ⎹ /       \ ⎸                     ⎹ /   |   \ ⎸
    //        11.----/9       6\----.7          11.----/9   |   6\----.7
    //          |   /           \   |     -->     |   / \   |   / \   |
    //          |  /             \  |             |  /   \  |  /   \  |
    //          | /               \ |             | /     \ | /     \ |
    //          |/_________________\|             |/_______\|/_______\|
    //          1 ⟍     3/\      ⟋ 2              1 ⟍     3/\      ⟋ 2
    //              ⟍   /  \   ⟋                      ⟍   /  \   ⟋       
    //                ⟍/____\⟋                          ⟍/____\⟋
    //                 4     5                           4     5

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
        Coordinate({ 5.00, 4.00, 0.00 }),   // 3
        Coordinate({ 4.00, 3.00, 0.00 }),   // 4
        Coordinate({ 6.00, 3.00, 0.00 }),   // 5
        Coordinate({ 6.50, 5.50, 0.00 }),   // 6
        Coordinate({ 8.00, 5.50, 0.00 }),   // 7
        Coordinate({ 6.50, 7.00, 0.00 }),   // 8
        Coordinate({ 3.50, 5.50, 0.00 }),   // 9
        Coordinate({ 3.50, 7.00, 0.00 }),   // 10
        Coordinate({ 2.00, 5.50, 0.00 }),   // 11
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({   0,  1,  2  }, Element::Type::Surface),  // 0
        Element({   3,  4,  5  }, Element::Type::Surface),  // 1
        Element({   2,  7,  6  }, Element::Type::Surface),  // 2
        Element({   0,  6,  8  }, Element::Type::Surface),  // 3
        Element({   0, 10,  9  }, Element::Type::Surface),  // 4
        Element({   1,  9, 11  }, Element::Type::Surface),  // 5
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({   3,  2,  6  }, Element::Type::Surface),  // 0
        Element({   3,  6,  0  }, Element::Type::Surface),  // 1

        Element({   3,  0,  9  }, Element::Type::Surface),  // 2
        Element({   3,  9,  1  }, Element::Type::Surface),  // 3

        Element({   3,  4,  5  }, Element::Type::Surface),  // 4
        Element({   2,  7,  6  }, Element::Type::Surface),  // 5
        Element({   0,  6,  8  }, Element::Type::Surface),  // 6
        Element({   0, 10,  9  }, Element::Type::Surface),  // 7
        Element({   1,  9, 11  }, Element::Type::Surface),  // 8
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}

TEST_F(SplicerTest, doNothingWithNormalTriangle)
{
    //                    0
    //                    /\
    //                   /  \
    //                  /    \
    //                 /      \
    //                /        \
    //               /          \
    //              /            \
    //             /              \
    //            /                \
    //           /__________________\
    //          1                   2


    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(0.0, 10.0, 11);
    inputMesh.coordinates = {
        Coordinate({ 5.00, 7.00, 0.00 }),   // 0
        Coordinate({ 2.00, 4.00, 0.00 }),   // 1
        Coordinate({ 8.00, 4.00, 0.00 }),   // 2
    };
    inputMesh.groups = { Group() };
    inputMesh.groups[0].elements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
    };

    Coordinates expectedCoordinates(inputMesh.coordinates);

    Mesh outputMesh;
    ASSERT_NO_THROW(outputMesh = Splicer{ inputMesh }.getMesh());

    Elements expectedElements = {
        Element({ 0, 1, 2 }, Element::Type::Surface),  // 0
    };

    assertCoordinatesListEquals(expectedCoordinates, outputMesh.coordinates);
    assertElementsListEquals(expectedElements, outputMesh.groups[0].elements);
}



}