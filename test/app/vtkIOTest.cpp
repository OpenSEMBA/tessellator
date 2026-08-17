#include <gtest/gtest.h>

#include "app/vtkIO.h"
#include "utils/GridTools.h"

using namespace meshlib::vtkIO;

class VTKIOTest : public ::testing::Test
{
};

TEST_F(VTKIOTest, readMeshFromSTL)
{
    std::string fn{"testData/cases/alhambra/alhambra.stl"};
    
    auto m{ readInputMesh(fn) };

    EXPECT_EQ(m.coordinates.size(), 584);
    EXPECT_EQ(m.groups.size(), 1);  
    EXPECT_EQ(m.countElems(), 1284);
}

TEST_F(VTKIOTest, exportAndReadMeshFromVTU)
{
    auto mSTL{ readInputMesh("testData/cases/alhambra/alhambra.stl") };
    exportMeshToVTU("testData/cases/alhambra/tmp_exported_alhambra.vtu", mSTL);
    auto mVTU{ readInputMesh("testData/cases/alhambra/tmp_exported_alhambra.vtu") };

    EXPECT_EQ(mSTL.coordinates.size(), mVTU.coordinates.size());
    EXPECT_EQ(mSTL.groups.size(), mVTU.groups.size());
    EXPECT_EQ(mSTL.countElems(), mVTU.countElems());
}

TEST_F(VTKIOTest, readElementTypes)
{
    auto m{ readInputMesh("testData/elementTypes.vtu") };

    EXPECT_EQ(m.coordinates.size(), 3);
    EXPECT_EQ(m.groups.size(), 1);
    EXPECT_EQ(m.groups[0].elements.size(), 3);
    EXPECT_TRUE(m.groups[0].elements[0].isNode());
    EXPECT_TRUE(m.groups[0].elements[1].isLine());
    EXPECT_TRUE(m.groups[0].elements[2].isTriangle());
}

TEST_F(VTKIOTest, exportAndReadHexahedron)
{
    meshlib::Mesh mesh;
    mesh.grid = meshlib::utils::GridTools::buildCartesianGrid(0.0, 1.0, 2);
    mesh.coordinates = {
        meshlib::Coordinate({0.0, 0.0, 0.0}), meshlib::Coordinate({1.0, 0.0, 0.0}),
        meshlib::Coordinate({1.0, 1.0, 0.0}), meshlib::Coordinate({0.0, 1.0, 0.0}),
        meshlib::Coordinate({0.0, 0.0, 1.0}), meshlib::Coordinate({1.0, 0.0, 1.0}),
        meshlib::Coordinate({1.0, 1.0, 1.0}), meshlib::Coordinate({0.0, 1.0, 1.0})};
    mesh.groups.resize(1);
    mesh.groups[0].elements.push_back(meshlib::Element(
        {0, 1, 2, 3, 4, 5, 6, 7}, meshlib::Element::Type::Volume));

    const auto filename = std::filesystem::temp_directory_path()
        / "tessellator_hexahedron_roundtrip.vtu";
    exportMeshToVTU(filename, mesh);
    const auto result = readInputMesh(filename);
    std::filesystem::remove(filename);

    ASSERT_EQ(1, result.countElems());
    EXPECT_TRUE(result.groups[0].elements[0].isHexahedron());
    EXPECT_EQ(mesh.coordinates, result.coordinates);
}

TEST_F(VTKIOTest, exportAndReadGroupNames)
{
    meshlib::Mesh mesh;
    mesh.coordinates = {
        meshlib::Coordinate({0.0, 0.0, 0.0}),
        meshlib::Coordinate({1.0, 0.0, 0.0})
    };
    mesh.groups = {
        meshlib::Group("first", {
            meshlib::Element({0}, meshlib::Element::Type::Node)}),
        meshlib::Group("second", {
            meshlib::Element({1}, meshlib::Element::Type::Node)})
    };

    const auto filename = std::filesystem::temp_directory_path()
        / "tessellator_group_names_roundtrip.vtu";
    exportMeshToVTU(filename, mesh);
    const auto result = readInputMesh(filename);
    std::filesystem::remove(filename);

    ASSERT_EQ(result.groups.size(), 2);
    EXPECT_EQ(result.groups[0].name, "first");
    EXPECT_EQ(result.groups[1].name, "second");
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
