#include <gtest/gtest.h>

#include "app/vtkIO.h"
#include "utils/GridTools.h"

using namespace meshlib::vtkIO;

class VTKIOTest : public ::testing::Test
{
};

TEST_F(VTKIOTest, readMeshFromSTL)
{
    std::string fn{"testData/alhambra.stl"};
    
    auto m{ readInputMesh(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);  
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKIOTest, readMeshFromVTK)
{
    std::string fn{"testData/alhambra.vtk"};
    
    auto m{ readInputMesh(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);  
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKIOTest, exportAndReadMeshFromVTU)
{
    auto mSTL{ readInputMesh("testData/alhambra.stl") };
    exportMeshToVTU("tmp_exported_alhambra.vtu", mSTL);
    auto mVTU{ readInputMesh("tmp_exported_alhambra.vtu") };

    EXPECT_EQ(mSTL.coordinates.size(), mVTU.coordinates.size());
    EXPECT_EQ(mSTL.groups.size(), mVTU.groups.size());  
    EXPECT_EQ(mSTL.countElems(), mVTU.countElems());
}

TEST_F(VTKIOTest, exportGridToVTU)
{
    meshlib::Grid grid;
    grid[0] = meshlib::utils::GridTools::linspace(-60, 60, 121);
    grid[1] = meshlib::utils::GridTools::linspace(-60, 60, 121);
    grid[2] = meshlib::utils::GridTools::linspace(-10, 10, 21);

    std::string fn{"tmp_exported_grid.vtu"};
    exportGridToVTU(fn, grid);

    auto exported{ readInputMesh(fn) };

    EXPECT_EQ(121+121+21, exported.countElems());
}