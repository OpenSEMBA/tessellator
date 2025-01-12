#include <gtest/gtest.h>

#include "app/vtkIO.h"

using namespace meshlib::vtkIO;

class VTKIOTest : public ::testing::Test
{
};

TEST_F(VTKIOTest, readsMeshFromVTK)
{
    std::string fn{"testData/alhambra.vtk"};
    
    auto m{ readMesh(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKIOTest, readMeshFromSTL)
{
    std::string fn{"testData/alhambra.stl"};
    
    auto m{ readMesh(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);  
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKIOTest, exportMeshToVTP)
{
    auto mesh{ readMesh("testData/alhambra.vtk") };
    
    std::string fn{"tmp_exported_alhambra.vtp"};
    exportMeshToVTP(fn, mesh);

    auto exported{ readMesh(fn) };

    EXPECT_EQ(exported, mesh);
}