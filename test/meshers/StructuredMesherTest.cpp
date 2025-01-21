#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "meshers/StructuredMesher.h"

#include "utils/Geometry.h"
#include "utils/GridTools.h"

namespace meshlib::meshers {

using namespace meshFixtures;
using namespace utils;
using namespace meshTools;


class StructuredMesherTest : public ::testing::Test {
public:
    static std::size_t countRepeatedElements(const Mesh& m)
    {
        std::set<std::set<CoordinateId>> verticesSets;
        std::set<CoordinateIds> lineVerticesSets;
        for (auto const& g : m.groups) {
            for (auto const& e : g.elements) {
                if (e.isLine()) {
                    lineVerticesSets.insert(e.vertices);
                }
                else {
                    verticesSets.insert(std::set<CoordinateId>(e.vertices.begin(), e.vertices.end()));  
                }
            }
        }
        return m.countElems() - verticesSets.size() - lineVerticesSets.size();
    }

    static void assertMeshEqual(const Mesh& leftMesh, const Mesh& rightMesh) {
        for (Axis axis : { X, Y, Z }) {
            auto& leftGridAxisPlanes = leftMesh.grid[axis];
            auto& rightGridAxisPlanes = rightMesh.grid[axis];

            EXPECT_EQ(leftGridAxisPlanes.size(), rightGridAxisPlanes.size());

            for (std::size_t plane = 0; plane != leftGridAxisPlanes.size(); ++plane) {
                EXPECT_EQ(leftGridAxisPlanes[plane], rightGridAxisPlanes[plane]);
            }
        }

        ASSERT_EQ(leftMesh.coordinates.size(), rightMesh.coordinates.size());

        for (CoordinateId id = 0; id < leftMesh.coordinates.size(); ++id) {
            for (auto axis : { X, Y, Z }) {
                EXPECT_EQ(leftMesh.coordinates[id][axis], rightMesh.coordinates[id][axis]);
            }
        }

        ASSERT_EQ(leftMesh.groups.size(), rightMesh.groups.size());

        for (std::size_t g = 0; g < leftMesh.groups.size(); ++g) {
            auto& leftGroup = leftMesh.groups[g];
            auto& rightGroup = leftMesh.groups[g];

            ASSERT_EQ(leftGroup.elements.size(), rightGroup.elements.size());

            for (std::size_t e = 0; e < leftGroup.elements.size(); ++e) {
                auto& resultElement = leftGroup.elements[e];
                auto& expectedElement = rightGroup.elements[e];
                EXPECT_EQ(resultElement.type, expectedElement.type);

                for (CoordinateId v = 0; v != resultElement.vertices.size(); ++v) {
                    EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
                }

            }
        }
    }
};

TEST_F(StructuredMesherTest, testStructuredLinesWithUniformGrid)
{

    float lowerCoordinateValue = -0.5;
    float upperCoordinateValue = 0.5;
    int numberOfCells = 4;
    float step = 0.25;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
    inputMesh.coordinates = {
        Coordinate({-0.475, -0.267, 0.452647}),
        Coordinate({ 0.4, 0.1, -0.2}),
    };
    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };

    Mesh expectedMesh;
    expectedMesh.grid = inputMesh.grid;
    expectedMesh.coordinates = {
        Coordinate({0.00, 1.00, 4.00}),     // 0
        Coordinate({1.00, 1.00, 4.00}),     // 1
        Coordinate({1.00, 1.00, 3.00}),     // 2
        Coordinate({1.00, 2.00, 3.00}),     // 3
        Coordinate({2.00, 2.00, 3.00}),     // 4
        Coordinate({2.00, 2.00, 2.00}),     // 5
        Coordinate({3.00, 2.00, 2.00}),     // 6
        Coordinate({3.00, 2.00, 1.00}),     // 7
        Coordinate({4.00, 2.00, 1.00}),     // 8
    };
    expectedMesh.groups.resize(1);
    expectedMesh.groups[0].elements = {
        Element({0}, Element::Type::Node),
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2}, Element::Type::Node),
        Element({2, 3}, Element::Type::Line),
        Element({3, 4}, Element::Type::Line),
        Element({4, 5}, Element::Type::Line),
        Element({5, 6}, Element::Type::Line),
        Element({6}, Element::Type::Node),
        Element({6, 7}, Element::Type::Line),
        Element({7, 8}, Element::Type::Line),
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = StructuredMesher(inputMesh, 2).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));

    assertMeshEqual(resultMesh, expectedMesh);
}


TEST_F(StructuredMesherTest, testStructuredLinesWithRectilinearGrid)
{
    float lowerCoordinateValue = -0.5;
    float upperCoordinateValue = 0.5;
    int numberOfCells = 4;
    float step = 0.25;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = Grid(
        {
            std::vector<CoordinateDir>({0.0, 1.0, 2.0}),
            std::vector<CoordinateDir>({-2.0, 0.0, 0.5}),
            std::vector<CoordinateDir>({0.0, 2.0, 6.0})
        }
    );
    inputMesh.coordinates = {
        Coordinate({0.13, -1.3, 0.1}),
        Coordinate({0.9, -0.5, 2.3}),
        Coordinate({1.4, 0.3, 5.5}),
    };

    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
    };


    Mesh expectedMesh;
    expectedMesh.grid = inputMesh.grid;
    expectedMesh.coordinates = {
        Coordinate({0.00, 0.00, 0.00}),     // 0
        Coordinate({0.00, 1.00, 0.00}),     // 1
        Coordinate({0.00, 1.00, 1.00}),     // 2
        Coordinate({1.00, 1.00, 1.00}),     // 3
        Coordinate({1.00, 1.00, 2.00}),     // 4
        Coordinate({1.00, 2.00, 2.00}),     // 5
    };
    expectedMesh.groups.resize(1);
    expectedMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2, 3}, Element::Type::Line),
        Element({3}, Element::Type::Node),
        Element({3, 4}, Element::Type::Line),
        Element({4, 5}, Element::Type::Line),
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = StructuredMesher(inputMesh, 2).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));

    assertMeshEqual(resultMesh, expectedMesh);
}

TEST_F(StructuredMesherTest, testTriNonUniformGridStructured)
{
    Mesh out;
    ASSERT_NO_THROW(out = StructuredMesher(buildTriNonUniformGridMesh(), 4).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(35, out.groups[0].elements.size());
    EXPECT_EQ(4, countMeshElementsIf(out, isQuad));
    EXPECT_EQ(22, countMeshElementsIf(out, isLine));
    EXPECT_EQ(9, countMeshElementsIf(out, isNode));
}



TEST_F(StructuredMesherTest, DISABLED_testStructuredTriangleWithUniformGrid)
{

    float lowerCoordinateValue = -0.5;
    float upperCoordinateValue = 0.5;
    int numberOfCells = 4;
    float step = 0.25;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
    inputMesh.coordinates = {
        Coordinate({-0.475, -0.15, -0.480}),
        Coordinate({ 0.475, -0.15, -0.480}),
        Coordinate({ 0.0, 0.15, 0.30}),
    };
    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {  
        Element({0, 1, 2}, Element::Type::Surface)
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = StructuredMesher(inputMesh, 2).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));
    EXPECT_EQ(48, resultMesh.groups[0].elements.size());
    EXPECT_EQ(10, countMeshElementsIf(resultMesh, isQuad));
    EXPECT_EQ(32, countMeshElementsIf(resultMesh, isLine)); 
    EXPECT_EQ(6, countMeshElementsIf(resultMesh, isNode));
}

}