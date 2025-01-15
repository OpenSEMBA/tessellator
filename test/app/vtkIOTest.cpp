#include <gtest/gtest.h>

#include "app/vtkIO.h"
#include "utils/GridTools.h"

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

TEST_F(VTKIOTest, exportGridToVTP)
{
    meshlib::Grid grid;
    grid[0] = meshlib::utils::GridTools::linspace(-60, 60, 121);
    grid[1] = meshlib::utils::GridTools::linspace(-60, 60, 121);
    grid[2] = meshlib::utils::GridTools::linspace(-10, 10, 21);

    std::string fn{"tmp_exported_grid.vtp"};
    exportGridToVTP(fn, grid);

    auto exported{ readMesh(fn) };

    EXPECT_EQ(121+121+21, exported.countElems());
}