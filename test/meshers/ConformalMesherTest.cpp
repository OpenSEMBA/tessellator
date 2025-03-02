#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

#include "meshers/ConformalMesher.h"
#include "utils/Geometry.h"

namespace meshlib::meshers {
using namespace meshFixtures;
using namespace utils::meshTools;

class ConformalMesherTest : public ::testing::Test {

};

TEST_F(ConformalMesherTest, cellsWithMoreThanAVertexPerEdge)
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

TEST_F(ConformalMesherTest, cellsWithInteriorDisconnectedPatches)
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
            Relative({1.25, 1.00, 1.50}),
            Relative({2.00, 1.50, 1.50}),
            Relative({1.00, 2.00, 1.50})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2})
        };
    }
    auto res = ConformalMesher::cellsWithInteriorDisconnectedPatches(m);

    EXPECT_EQ(1, res.size());
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