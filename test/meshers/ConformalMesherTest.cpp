#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

#include "meshers/ConformalMesher.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

#if APP_LOADED
    #include "app/vtkIO.h"
#endif

namespace meshlib::meshers {
using namespace meshFixtures;
using namespace utils::meshTools;

#if APP_LOADED
    using namespace vtkIO;
#endif

class ConformalMesherTest : public ::testing::Test {
protected:

#if APP_LOADED
    Mesh launchConformalMesher(const std::string& inputFilename, const Mesh& inputMesh)
    {
        ConformalMesherOptions opts;
        opts.snapperOptions.edgePoints = 3;
        opts.snapperOptions.forbiddenLength = 0.3;
        
        ConformalMesher mesher{inputMesh, opts};

        Mesh res = mesher.mesh();
    
        std::filesystem::path outputFolder = getFolder(inputFilename);
        auto basename = getBasename(inputFilename);
        exportMeshToVTU(outputFolder / (basename + ".tessellator.cmsh.vtk"), res);
        exportGridToVTU(outputFolder / (basename + ".tessellator.grid.vtk"), res.grid);

        return res;
    }
#endif
};

TEST_F(ConformalMesherTest, findsCellOccupiedByDifferentGroups)
{
    Mesh mesh;
    mesh.grid = buildUnitLengthGrid(0.5);
    mesh.coordinates = {
        Relative({0.25, 0.25, 0.25}),
        Relative({0.75, 0.75, 0.75}),
        Relative({1.25, 0.25, 0.25})
    };
    mesh.groups = {
        Group("first", {Element({0}, Element::Type::Node)}),
        Group("second", {Element({1}, Element::Type::Node)}),
        Group("third", {Element({2}, Element::Type::Node)})
    };

    const auto result = ConformalMesher::cellsSharedByGroups(mesh);

    EXPECT_EQ(result, std::set<Cell>({Cell({0, 0, 0})}));
}

TEST_F(ConformalMesherTest, includesBothCellsWhenGroupsMeetOnCellFace)
{
    Mesh mesh;
    mesh.grid = buildUnitLengthGrid(0.5);
    mesh.coordinates = {
        Relative({1.0, 0.25, 0.25}),
        Relative({1.0, 0.75, 0.75})
    };
    mesh.groups = {
        Group("first", {Element({0}, Element::Type::Node)}),
        Group("second", {Element({1}, Element::Type::Node)})
    };

    const auto result = ConformalMesher::cellsSharedByGroups(mesh);

    EXPECT_EQ(result, std::set<Cell>({Cell({0, 0, 0}), Cell({1, 0, 0})}));
}

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
    // Triangles forming a patch in a cell face with a single boundary 
    // crossing the cell face.
    //  1 ----- 2
    //  | ____/ |
    //  0 ___   |
    //    \   \ |
    //      4 - 3
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

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_6)
{
    // Triangles forming a patch in a cell face with a single boundary 
    // crossing the cell face. And two triangles across cell.
    //  1 ----- 2
    //  | ____/ |
    // 0,5 ___  |
    //    \   \ |
    //    4,6 - 3
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.1); // 10 x 10 x 10 grid
        m.coordinates = {
            Relative({0.0, 0.5, 1.0}), // 0 
            Relative({0.0, 1.0, 1.0}), // 1
            Relative({1.0, 1.0, 1.0}), // 2
            Relative({1.0, 0.0, 1.0}), // 3
            Relative({0.5, 0.0, 1.0}), // 4
            Relative({0.0, 0.5, 2.0}), // 5
            Relative({0.5, 0.0, 2.0}), // 6
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 3, 0}),
            Element({3, 4, 0}),
            Element({0, 4, 5}),
            Element({5, 4, 6})
        };
    }

    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_7)
{
    Mesh m;
    {
        m.grid = buildUnitLengthGrid(0.01); // 100 x 100 x 100 grid
        m.coordinates = {
            Relative({34.0, 35.0, 1.0}), // 0 
            Relative({35.0, 35.0, 1.0}), // 1
            Relative({34.0, 35.5, 1.0}), // 2
            Relative({35.0, 36.0, 1.0}), // 3
            Relative({34.3, 36.0, 1.0}), // 4
            Relative({34.3, 36.0, 2.0}), // 5
            Relative({34.0, 35.5, 2.0}), // 6
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 2, 1}),
            Element({1, 2, 3}),
            Element({2, 4, 3}),
            Element({2, 5, 4}),
            Element({5, 2, 6})
        };
    }

    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(0, res.size());
}

TEST_F(ConformalMesherTest, cellsWithMoreThanAPathPerFace_8)
{
    // 2 Disconnected triangles on cell face.
    //  1 ----- 2
    //  | ____/ 
    //  0 ___   
    //    \   \ 
    //      4 - 3
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
            Element({3, 4, 0}),
        };
    }
    
    auto res = ConformalMesher::cellsWithMoreThanAPathPerFace(m);

    EXPECT_EQ(2, res.size());
}

#if APP_LOADED

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
        // utils::meshTools::convertToAbsoluteCoordinates(dbgMesh);
        exportMeshToVTU("testData/cases/sphere/sphere.breaksRuleNo2.vtk", dbgMesh);
    }
} 

TEST_F(ConformalMesherTest, alhambra)
{
    // Input
    const std::string inputFilename = "testData/cases/alhambra/alhambra.stl";
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

#endif

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
