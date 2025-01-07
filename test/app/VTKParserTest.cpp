#include <gtest/gtest.h>

#include "app/VTKParser.h"

using namespace meshlib;

class VTKParserTest : public ::testing::Test
{
};

TEST_F(VTKParserTest, readsVTK)
{
    std::string fn{"testData/alhambra.vtk"};
    
    auto m{ readVTK(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKParserTest, readSTL)
{
    std::string fn{"testData/alhambra.stl"};
    
    auto m{ readVTK(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);
    EXPECT_EQ(m.countElems(), 1284);
}
