#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Snapper.h"
#include "Slicer.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace core;
using namespace utils;
using namespace meshFixtures;

class SnapperTest : public ::testing::Test {
protected:
};

TEST_F(SnapperTest, similar_results_for_each_plane)
{
    SnapperOptions opts;
    opts.edgePoints = 7;
    opts.forbiddenLength = 0.1;

    for (auto z: {0.0, 1.0}) {

        Mesh m;
        m.grid = buildUnitLengthGrid(1.0);
        m.coordinates = {
            Relative({0.80, 1.00, z}),
            Relative({1.00, 1.00, z}),
            Relative({0.80, 0.00, z}),
            Relative({1.00, 0.00, z})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 1, 3})
        };
                
        auto res = Snapper(m, opts).getMesh();
        EXPECT_EQ(res.coordinates, m.coordinates);

    }
}

TEST_F(SnapperTest, triangles_convert_to_lines)
{
    SnapperOptions opts;
    opts.edgePoints = 0;
    opts.forbiddenLength = 0.5;
   
    Mesh m;
    m.grid = buildUnitLengthGrid(1.0);
    m.coordinates = {
        Relative({0.0, 0.0, 0.0}),
        Relative({0.5, 0.0, 0.0}),
        Relative({1.0, 0.0, 0.0})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}),
    };
                
    auto res = Snapper(m, opts).getMesh();
    
    Relatives expectedCoords = {Relative({0.0, 0.0, 0.0}), Relative({1.0, 0.0, 0.0})};
    Element expectedElement({0, 1}, Element::Type::Line);    
    
    EXPECT_EQ(expectedCoords, res.coordinates);
    ASSERT_EQ(1, res.groups.size());
    ASSERT_EQ(1, res.groups[0].elements.size());
    EXPECT_EQ(expectedElement, res.groups[0].elements[0]);
}

TEST_F(SnapperTest, triangles_convert_to_nodes)
{
    SnapperOptions opts;
    opts.edgePoints = 0;
    opts.forbiddenLength = 0.5;
   
    Mesh m;
    m.grid = buildUnitLengthGrid(1.0);
    m.coordinates = {
        Relative({0.0, 0.0, 0.0}),
        Relative({0.1, 0.0, 0.0}),
        Relative({0.2, 0.0, 0.0})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}),
    };
                
    auto res = Snapper(m, opts).getMesh();
    
    Relatives expectedCoords = {Relative({0.0, 0.0, 0.0})};
    Element expectedElement({0}, Element::Type::Node);    
    
    EXPECT_EQ(expectedCoords, res.coordinates);
    ASSERT_EQ(1, res.groups.size());
    ASSERT_EQ(1, res.groups[0].elements.size());
    EXPECT_EQ(expectedElement, res.groups[0].elements[0]);
}

TEST_F(SnapperTest, DISABLED_redundant_lines_are_removed)
{
    
}
