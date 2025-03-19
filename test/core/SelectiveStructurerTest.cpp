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

    class SelectiveStructurerTest : public ::testing::Test {
        protected:
    };

TEST_F(SelectiveStructurerTest, modifyJustOneCoordinateInACell)
{
    // *---------*      *---------*
    // |         |      |         |
	// |         |      |         |
    // |     _-1 |  ->  |         |
	// |  0-‾    |      |  0-__   |
    // *---------*      *------‾‾-1

    float lowerCoordinateValue = 0.0;
    float upperCoordinateValue = 10.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Relative({ 0.3, 0.3, 0.0 }),
        Relative({ 0.7, 0.4, 0.0 }),
    };
    mesh.groups = { Group()};
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    
    Relatives expectedRelatives = {
        Relative({0.3, 0.3, 0.0}),
        Relative({1.0, 0.0, 0.0})
    };

    Elements expectedElements = {
        Element({0, 1}, Element::Type::Line)
    };

    auto resultCoordinate = Structurer{ mesh }.calculateStructuredCell(mesh.coordinates[1]);

    for (std::size_t axis=0; axis < 3; ++axis) {
        EXPECT_EQ(resultCoordinate[axis], expectedRelatives[1][axis]);
        EXPECT_EQ(mesh.coordinates[0][axis], expectedRelatives[0][axis]);
        mesh.coordinates[1][axis] = resultCoordinate[axis];
    }

    for (std::size_t axis=0; axis < 3; ++axis) {
        for (std::size_t rel=0; rel < mesh.coordinates.size(); ++rel) {
            EXPECT_EQ(mesh.coordinates[rel][axis], expectedRelatives[rel][axis]);
        }
    }


    auto& meshGroup = mesh.groups[0];
    auto& expectedElement = expectedElements[0];

    EXPECT_TRUE(meshGroup.elements[0].isLine());
    for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
        EXPECT_EQ(meshGroup.elements[0].vertices[v], expectedElement.vertices[v]);
    }
}

TEST_F(SelectiveStructurerTest, modifyAllCoordinatesAndCompareToStructurer)
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
        Relative({ 0.3, 0.3, 0.0 }),
        Relative({ 0.7, 0.4, 0.0 }),
        Relative({ 0.3, 0.7, 0.0 }),
    };
    mesh.groups = { Group(), Group()};
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line)
    };
    mesh.groups[1].elements = {
        Element({0, 2}, Element::Type::Line)
    };

    auto expectedMesh = Structurer{ mesh }.getMesh();
    
    std::vector<decltype(Structurer{ mesh }.calculateStructuredCell(mesh.coordinates[0]))> resultCoordinates(mesh.coordinates.size());
    for (std::size_t rel=0; rel < mesh.coordinates.size(); ++rel) {
        resultCoordinates[rel] = Structurer{ mesh }.calculateStructuredCell(mesh.coordinates[rel]);
    }

    Mesh resultMesh = mesh;
    for(std::size_t rel=0; rel < mesh.coordinates.size(); ++rel) {
        for(std::size_t axis=0; axis < 3; ++axis) {
            resultMesh.coordinates[rel][axis] = resultCoordinates[rel][axis];
        }
    }
    
    ASSERT_EQ(resultMesh.coordinates.size(), expectedMesh.coordinates.size());
    ASSERT_EQ(resultMesh.groups.size(), expectedMesh.groups.size());
    for (std::size_t g = 0; g < resultMesh.groups.size(); ++g) {
        ASSERT_EQ(resultMesh.groups[g].elements.size(), 1);
    }                                 

    for (std::size_t g = 0; g < expectedMesh.coordinates.size(); ++g) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[g][axis], expectedMesh.coordinates[g][axis]);
        }
    }
    

    for (std::size_t g = 0; g < expectedMesh.groups.size(); ++g) {
        auto& resultGroup = resultMesh.groups[g];
        auto& expectedElement = expectedMesh.groups[g];

        EXPECT_TRUE(resultGroup.elements[0].isLine());
        for (std::size_t v = 0; v < expectedElement.elements[0].vertices.size(); ++v) {
            EXPECT_EQ(resultGroup.elements[0].vertices[v], expectedElement.elements[0].vertices[v]);
        }
    }
}

TEST_F(SelectiveStructurerTest, modifyCoordinateOfASpecificCell)
{

    // *-------------*-------------*          *---------------*---------------* 
    // |             |             |          |               |               | 
    // |             |             |          |               |               | 
    // |             |        _2   |  ->      |               |          _2   | 
    // |             |     _-‾     |          |               |       _-‾     | 
    // |             |  _-‾        |          |               |    _-‾        | 
    // |     0-------1-‾           |          |               | _-‾           | 
    // *-------------*-------------*          0===============1‾--------------*
    //
    
    float lowerCoordinateValue = -5.0;
    float upperCoordinateValue = 5.0;
    int numberOfCells = 3;
    float step = 5.0;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells - 1) == step);

    Cell structuredCell = {0, 0, 0};
    
    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells);
    mesh.coordinates = {
        Relative({ 0.4, 0.1, 0.7 }), // 0 First Segment, First Point
        Relative({ 1.0, 0.1, 0.7 }), // 1 First Segment, Second Point
        Relative({ 1.8, 0.6, 0.7 }), // 2 Second Segment, Final Point
    };

    mesh.groups.resize(1);
    mesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
    };

    Relatives expectedRelatives = {
        Relative({ 0.0, 0.0, 1.0 }), // 0 First Segment, First Point
        Relative({ 1.0, 0.0, 1.0 }), // 1 First Segment, Final Point, Second Segment, First Point
        Relative({ 1.8, 0.6, 0.7 }), // 2 Second Segment, Final Point
    };

    Elements expectedElements = {
            Element({0, 1}, Element::Type::Line),
            Element({1, 2}, Element::Type::Line),
    };

    std::vector<Coordinate> resultCoordinates(mesh.coordinates.size());

    for (std::size_t rel=0; rel < mesh.coordinates.size(); ++rel) {

        bool coordIsInsideCell = true;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!(structuredCell[axis] <= mesh.coordinates[rel][axis] && mesh.coordinates[rel][axis] <= structuredCell[axis] + 1)) {
                coordIsInsideCell = false;
                break;
            }
        }

        if(coordIsInsideCell) {
            auto resultCell = Structurer{ mesh }.calculateStructuredCell(mesh.coordinates[rel]);
            for(std::size_t axis=0; axis < 3; ++axis){
                resultCoordinates[rel][axis] = resultCell[axis];
            }
        } else {
            resultCoordinates[rel] = mesh.coordinates[rel];
        }
    }

    Mesh resultMesh = mesh;
    for(std::size_t rel=0; rel < mesh.coordinates.size(); ++rel) {
        for(std::size_t axis=0; axis < 3; ++axis) {
            resultMesh.coordinates[rel][axis] = resultCoordinates[rel][axis];
        }
    }

    ASSERT_EQ(resultMesh.coordinates.size(), expectedRelatives.size());
    ASSERT_EQ(resultMesh.groups.size(), 1);
    ASSERT_EQ(resultMesh.groups[0].elements.size(), expectedElements.size());

    for (std::size_t i = 0; i < expectedRelatives.size(); ++i) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(resultMesh.coordinates[i][axis], expectedRelatives[i][axis]);
        }
    }

    ASSERT_TRUE(resultMesh.groups[0].elements[0].isLine());
    ASSERT_TRUE(resultMesh.groups[0].elements[1].isLine());

    for (std::size_t e = 0; e < expectedElements.size(); ++e) {
        auto& resultElement = resultMesh.groups[0].elements[e];
        auto& expectedElement = expectedElements[e];

        for (std::size_t v = 0; v < expectedElement.vertices.size(); ++v) {
            EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v]);
        }
    }

}

}