#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

#include "meshers/ConformalMesher.h"
#include "utils/Geometry.h"
#include "app/vtkIO.h"
#include "utils/MeshTools.h"

namespace meshlib::meshers {
using namespace meshFixtures;
using namespace utils::meshTools;
using namespace vtkIO;

class ConformalMesherTest : public ::testing::Test {
protected:
    Mesh launchConformalMesher(const std::string& inputFilename, const Mesh& inputMesh)
    {
        ConformalMesherOptions opts;
        opts.snapperOptions.edgePoints = 0;
        opts.snapperOptions.forbiddenLength = 0.0;
        
        ConformalMesher mesher{inputMesh, opts};

        Mesh res = mesher.mesh();
    
        std::filesystem::path outputFolder = getFolder(inputFilename);
        auto basename = getBasename(inputFilename);
        exportMeshToVTU(outputFolder / (basename + ".tessellator.cmsh.vtk"), res);
        exportGridToVTU(outputFolder / (basename + ".tessellator.grid.vtk"), res.grid);

        return res;
    }
};

TEST_F(ConformalMesherTest, cellsWithMoreThanAVertexPerEdge_1)
{
    // This is non-conformal.
    //  4  _
    //  |\\ \
    //  | \ \ \
    //  0=1==2=3
    
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1);
        m.coordinates = {
            Relative({1.00, 1.00, 1.00}),
            Relative({1.25, 1.00, 1.00}),
            Relative({1.75, 1.00, 1.00}),
            Relative({2.00, 1.00, 1.00}),
            Relative({1.00, 2.00, 1.00})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 4}),
            Element({1, 2, 4}),
            Element({2, 3, 4})
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAVertexInsideEdge(m);

    EXPECT_EQ(4, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAVertexPerEdge_2)
{
    // Triangles forming a patch in a cell face with a single boundary crossing the cell face.
    // It is conformal.
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1); // 10 x 10 x 10 grid
        m.coordinates = {
            Relative({0.0, 0.5, 1.0}),
            Relative({0.0, 1.0, 1.0}),
            Relative({1.0, 1.0, 1.0}),
            Relative({1.0, 0.0, 1.0}),
            Relative({0.5, 0.0, 1.0}),
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 3, 0}),
            Element({3, 4, 0}),
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAVertexInsideEdge(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_1)
{
    // Triangle in a cell face with vertices on edges.
    // Is non-conformal.
    //  2-- 
    //   \ -- 1 
    //    \ / 
    //     0  
    
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1);
        m.coordinates = {
            Relative({1.25, 1.00, 1.00}),
            Relative({2.00, 1.50, 1.00}),
            Relative({1.00, 2.00, 1.00})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2})
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(2, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_2)
{
    // Triangles forming a patch in a cell face with a single boundary crossing the cell face.
    // It is conformal.
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1); // 10 x 10 x 10 grid
        m.coordinates = {
            Relative({0.0, 0.5, 1.0}),
            Relative({0.0, 1.0, 1.0}),
            Relative({1.0, 1.0, 1.0}),
            Relative({1.0, 0.0, 1.0}),
            Relative({0.5, 0.0, 1.0}),
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 3, 0}),
            Element({3, 4, 0}),
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_3)
{
    // Triangle in a cell face with two vertices in corner and on edge.
    // Is conformal.
    //  2--1 
    //  | / 
    //  0  
    
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1);
        m.coordinates = {
            Relative({1.00, 1.00, 1.00}),
            Relative({1.50, 2.00, 1.00}),
            Relative({1.00, 2.00, 1.00})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2})
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_4)
{
    // Patch with two triangles on face and two triangles within the cell.
    // Is conformal.
    //  1---2 
    //  |  /   - -  -5
    //  |/ _ - -4
    //  0 ----- 3
    
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1); // 10 x 10 x 10 grid
        m.coordinates = {
            Relative({1.0, 1.0, 1.0}),
            Relative({1.0, 2.0, 1.0}),
            Relative({1.5, 2.0, 1.0}),
            Relative({2.0, 1.0, 1.0}),
            Relative({2.0, 1.5, 1.0}),
            Relative({1.5, 1.5, 1.5}),
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 5, 0}),
            Element({0, 5, 4}),
            Element({4, 3, 0}),
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(2, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_5)
{
    // Patch with one triangles on face and one triangles within the cell.
    // Is conformal.
    //  1---2 
    //  |  /   - -  -5
    //  |/ _ ---
    //  0 
    
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1); // 10 x 10 x 10 grid
        m.coordinates = {
            Relative({1.0, 1.0, 1.0}),
            Relative({1.0, 2.0, 1.0}),
            Relative({1.5, 2.0, 1.0}),
            Relative({1.5, 1.5, 1.5}),
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 3, 0}),
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithInteriorDisconnectedPatches_1)
{
    // Isolated patches in a cell with vertices not touching any edge.
    // Is non-conformal.
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1);
        m.coordinates = {
            Relative({1.25, 1.20, 1.50}),
            Relative({1.70, 1.50, 1.50}),
            Relative({1.20, 1.80, 1.50})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2})
        };
    }
    auto res = ConformalMesher::cellsWithInteriorDisconnectedPatches(m);

    EXPECT_EQ(1, res.size());
}

TEST_F(ConformalMesherTest, cellsWithInteriorDisconnectedPatches_2)
{
    // Coplanar triangles are conformal.
    //  1 -- 2 
    //  |  / |
    //  | /  |
    //  0 -- 3
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1);
        m.coordinates = {
            Relative({0.00, 0.00, 1.50}),
            Relative({0.00, 1.00, 1.50}),
            Relative({1.00, 1.00, 1.50}),
            Relative({1.00, 0.00, 1.50})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 3, 0})
        };
    }
    auto res = ConformalMesher::cellsWithInteriorDisconnectedPatches(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, sphere)
{
    // Input
    const std::string inputFilename = "testData/cases/sphere/sphere.stl";
    auto inputMesh = vtkIO::readInputMesh(inputFilename);

    for (auto x: {X,Y,Z}) {
        inputMesh.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    Mesh mesh;
    EXPECT_NO_THROW(mesh = launchConformalMesher(inputFilename, inputMesh));

    // For Debugging.
    mesh.coordinates = utils::GridTools{mesh.grid}.absoluteToRelative(mesh.coordinates);
    {
        auto cells = ConformalMesher::cellsWithMoreThanAVertexInsideEdge(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/sphere/sphere.breaksRuleNo1.vtk", dbgMesh);
    }
    {
        auto cells = ConformalMesher::cellsWithMoreThanAPathPerFace(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/sphere/sphere.breaksRuleNo2.vtk", dbgMesh);
    }
    {
        auto cells = ConformalMesher::cellsWithInteriorDisconnectedPatches(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/sphere/sphere.breaksRuleNo3.vtk", dbgMesh);
    }
} 

TEST_F(ConformalMesherTest, alhambra)
{
    // Input
    const std::string inputFilename = "testData/cases/alhambra/alhambra.vtk";
    auto inputMesh = vtkIO::readInputMesh(inputFilename);

    inputMesh.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    inputMesh.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    inputMesh.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    
    // Mesh
    auto mesh = launchConformalMesher(inputFilename, inputMesh);

    // For Debugging.
    mesh.coordinates = utils::GridTools{mesh.grid}.absoluteToRelative(mesh.coordinates);
    {
        auto cells = ConformalMesher::cellsWithMoreThanAVertexInsideEdge(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/alhambra/alhambra.breaksRuleNo1.vtk", dbgMesh);
    }
    {
        auto cells = ConformalMesher::cellsWithMoreThanAPathPerFace(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/alhambra/alhambra.breaksRuleNo2.vtk", dbgMesh);
    }
    {
        auto cells = ConformalMesher::cellsWithInteriorDisconnectedPatches(mesh);
        auto dbgMesh = utils::meshTools::buildMeshFromSelectedCells(mesh, cells);
        utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/alhambra/alhambra.breaksRuleNo3.vtk", dbgMesh);
    }
}

TEST_F(ConformalMesherTest, cone)
{
    // Input
    const std::string inputFilename = "testData/cases/cone/cone.stl";
    auto inputMesh = vtkIO::readInputMesh(inputFilename);

    inputMesh.grid[X] = utils::GridTools::linspace(-2.0,  2.0,  41); 
    inputMesh.grid[Y] = utils::GridTools::linspace(-2.0,  2.0,  41); 
    inputMesh.grid[Z] = utils::GridTools::linspace(-1.0, 11.0, 121);
    
    // Mesh
    auto mesh = launchConformalMesher(inputFilename, inputMesh);
}

TEST_F(ConformalMesherTest, thinCylinder)
{
    // Input
    const std::string inputFilename = "testData/cases/thinCylinder/thinCylinder.stl";
    auto inputMesh = vtkIO::readInputMesh(inputFilename);

    inputMesh.grid[X] = utils::GridTools::linspace(-1.0,  1.0, 21); 
    inputMesh.grid[Y] = utils::GridTools::linspace(-1.0,  1.0, 21); 
    inputMesh.grid[Z] = utils::GridTools::linspace(-1.0,  2.0, 31);
    
    // Mesh
    auto mesh = launchConformalMesher(inputFilename, inputMesh);

    EXPECT_NE(0, mesh.countElems());
}

// TEST_F(ConformalMesherTest, plane45_size05_grid_adapted) 
// {
//     ConformalMesher mesher(buildPlane45Mesh(0.5));
    
//     Mesh out;

//     ASSERT_NO_THROW(out = mesher.mesh());
//     EXPECT_EQ(8, out.groups[0].elements.size());
// }

// TEST_F(ConformalMesherTest, plane45_size05_grid_raw)
// {
//     ConformalMesher mesher(buildPlane45Mesh(0.5));

//     Mesh out;

//     ASSERT_NO_THROW(out = mesher.mesh());
//     EXPECT_EQ(12, out.groups[0].elements.size());
// }

// TEST_F(ConformalMesherTest, plane45_size025_grid_adapted) {

//     ConformalMesher mesher(buildPlane45Mesh(0.25));

//     Mesh out;

//     ASSERT_NO_THROW(out = mesher.mesh());
//     EXPECT_EQ(32, countMeshElementsIf(out, isTriangle));
// }

// TEST_F(ConformalMesherTest, plane45_size025_grid_raw) 
// {

//     ConformalMesher mesher(buildPlane45Mesh(0.25));

//     Mesh out;

//     ASSERT_NO_THROW(out = mesher.mesh());
//     EXPECT_EQ(40, countMeshElementsIf(out, isTriangle));
//     EXPECT_TRUE(false); // WIP.
// }

// TEST_F(ConformalMesherTest, slab_surface_treat_as_volume)
// {
//     ConformalMesherOptions opts;
//     opts.snapperOptions.forbiddenLength = 0.25;
//     opts.volumeGroups = { 0 };
    
//     Mesh p;
//     ASSERT_NO_THROW(p = ConformalMesher(buildSlabSurfaceMesh(1.0, 0.01), opts).mesh());
    
//     EXPECT_EQ(4, countMeshElementsIf(p, isTriangle));
// }



}