#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "filler/Filler.h"
#include "Slicer.h"

using namespace meshlib;
using namespace tessellator;
using namespace filler;
using namespace meshFixtures;

class FillerTest : public ::testing::Test {
public:
    static const std::size_t X{ 0 };
    static const std::size_t Y{ 1 };
    static const std::size_t Z{ 2 };
    
    static Mesh buildParalelogramMesh()
    {
        ///    * --- *
        ///  / 11 --/10
        /// * /-- * /|
        /// 8 -|- 9  |
        /// |  5 --- 6
        /// |/ |  |/ |
        /// 4 --- 7  |
        /// |  |  |  |
        /// |  1 -|- 2
        /// | /   | /
        /// 0 --- 3  
        Grid grid;
        grid[0] = { 0.0, 1.0 };
        grid[1] = { 0.0, 1.0 };
        grid[2] = { 0.0, 1.0, 2.0 };
        std::vector<Coordinate> coords(12);

        coords[ 0] = Coordinate{ {0.00, 0.00, 0.00} };
        coords[ 1] = Coordinate{ {1.00, 0.00, 0.00} };
        coords[ 2] = Coordinate{ {0.00, 1.00, 0.00} };
        coords[ 3] = Coordinate{ {1.00, 1.00, 0.00} };
                
        coords[ 4] = Coordinate{ {0.00, 0.00, 1.00} };
        coords[ 5] = Coordinate{ {1.00, 0.00, 1.00} };
        coords[ 6] = Coordinate{ {0.00, 1.00, 1.00} };
        coords[ 7] = Coordinate{ {1.00, 1.00, 1.00} };
                
        coords[ 8] = Coordinate{ {0.00, 0.00, 1.50} };
        coords[ 9] = Coordinate{ {1.00, 0.00, 1.50} };
        coords[10] = Coordinate{ {0.00, 1.00, 1.50} };
        coords[11] = Coordinate{ {1.00, 1.00, 1.50} };

        Element tri;
        tri.type = Element::Type::Surface;
        std::vector<Group> groups = { Group({std::vector<Element>(20, tri)}) };
        groups[0].elements[0].vertices = { 0, 2, 1 };
        groups[0].elements[1].vertices = { 1, 2, 3 };

        groups[0].elements[2].vertices = { 0, 1, 4 };
        groups[0].elements[3].vertices = { 1, 5, 4 };
        groups[0].elements[4].vertices = { 1, 7, 5 };
        groups[0].elements[5].vertices = { 1, 3, 7 };
        groups[0].elements[6].vertices = { 0, 4, 6 };
        groups[0].elements[7].vertices = { 0, 6, 2 };
        groups[0].elements[8].vertices = { 3, 2, 6 };
        groups[0].elements[9].vertices = { 3, 6, 7 };

        groups[0].elements[10].vertices = { 4, 5, 8 };
        groups[0].elements[11].vertices = { 5, 9, 8 };
        groups[0].elements[12].vertices = { 5, 11, 9 };
        groups[0].elements[13].vertices = { 5, 7, 11 };
        groups[0].elements[14].vertices = { 4, 8, 10 };
        groups[0].elements[15].vertices = { 4, 10, 6 };
        groups[0].elements[16].vertices = { 7, 6, 10 };
        groups[0].elements[17].vertices = { 7, 10, 11 };

        groups[0].elements[18].vertices = { 8, 9, 10 };
        groups[0].elements[19].vertices = { 9, 11, 10 };

        return Mesh{ grid, coords, groups };
    }

    static void scale(Coordinates& cs, const VecD& sF)
    {
        std::transform(
            cs.begin(), cs.end(),
            cs.begin(),
            [&](auto& c) {
                for (auto d{ 0 }; d < 3; ++d) {
                    c[d] *= sF[d];
                }
                return c;
            }
        );
    }
    static void addOffset(Coordinates& cs, Coordinate offset)
    {
        std::transform(
            cs.begin(), cs.end(),
            cs.begin(),
            [&](auto& c) { return c + offset; }
        );
    }

    static std::size_t countPWHs(const FaceFilling& f) 
    {
        std::size_t res{ 0 };
        for (const auto& prT : f.tris) {
            res += prT.second.size();
        }
        return res;
    }

    static std::size_t countLins(const FaceFilling& f)
    {
        std::size_t res{ 0 };
        for (const auto& prT : f.lins) {
            res += prT.second.size();
        }
        return res;
    }

    static std::size_t countLins(const EdgeFilling& f)
    {
        std::size_t res{ 0 };
        for (const auto& prT : f.lins) {
            res += prT.second.size();
        }
        return res;
    }

    static bool hasZeroLengthSegments(const FaceFilling& f)
    {
        for (const auto& [pr, polylines] : f.lins) {
            for (const auto& polyline : polylines) {
                for (auto it{ polyline.begin() }; std::next(it) != polyline.end(); ++it) {
                    if (*it == *std::next(it)) {
                        return true;
                    }
                }               
            }
        }
        return false;
    }

    static Mesh toRelative(const Mesh& m) 
    {
        auto r{ m };
        r.coordinates = 
            utils::GridTools{ m.grid }.absoluteToRelative(m.coordinates);
        return r;
    }

    static bool allAreSimple(const FaceFilling& ff)
    {
        for (const auto [pr, ss] : ff.tris) {
            if (!ss.isSimple()) {
                return false;
            }
        }
        return true;
    }
};

TEST_F(FillerTest, tet_stepSize1)
{
    Filler f{ Slicer{buildTetSurfaceMesh(1.0)}.getMesh() };

    EXPECT_EQ(3, f.getMeshFilling().countTriangles());

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), X })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ Cell({1, 0, 0}), X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Y })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Z })));
}

TEST_F(FillerTest, tet_stepSize1_no_slicing)
{
    Filler f{ toRelative(buildTetSurfaceMesh(1.0)) };

    EXPECT_EQ(3, f.getMeshFilling().countTriangles());

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), X })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ Cell({1, 0, 0}), X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Y })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Z })));
}

TEST_F(FillerTest, singleTri_no_zero_length_edge)
{
    Filler f{ Slicer{buildSingleCellTriMesh(0.2)}.getMesh() };
    
    EXPECT_FALSE(
        hasZeroLengthSegments(
            f.getFaceFilling({ Cell({2, 2, 4}), Z }) ) );
}

TEST_F(FillerTest, tet_stepSize0c5x)
{
    auto m{ buildTetSurfaceMesh(1.0) };
    m.grid[0] = { 0.0, 0.5, 1.0 };

    auto in{ Slicer{m}.getMesh() };
    auto out{ Filler{in}.getMeshFilling() };

    EXPECT_EQ(in.grid, out.grid);
    EXPECT_EQ(4, out.countElems());
}

TEST_F(FillerTest, tet_stepSize0c5x_no_slicing)
{
    Mesh m;
    {
        auto m2{ buildTetSurfaceMesh(1.0) };
        m2.grid[0] = { 0.0, 0.5, 1.0 };
        m = toRelative(m2);
    }

    auto out{ Filler{m}.getMeshFilling() };

    EXPECT_EQ(m.grid, out.grid);
    EXPECT_EQ(4, out.countElems());
}

TEST_F(FillerTest, cube_stepSize1_cell_000)
{
    Filler f{ Slicer{buildCubeSurfaceMesh(1.0) }.getMesh() };
    
    EXPECT_EQ(12, f.getMeshFilling().countTriangles());

    Cell c({ 0, 0, 0 });
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ c, X })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ c, Y })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ c, Z })));
}

TEST_F(FillerTest, cube_stepSize1_cell_111)
{
    Filler f{ Slicer{buildCubeSurfaceMesh(1.0) }.getMesh() };

    Cell c({ 1, 1, 1 });
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, Y })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, Z })));
}

TEST_F(FillerTest, cube_stepSize1_cell_211)
{
    Filler f{ Slicer{buildCubeSurfaceMesh(1.0) }.getMesh() };

    Cell c({ 2, 1, 1 });
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, X })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ c, Y })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ c, Z })));
}

TEST_F(FillerTest, cube_stepSize0c5)
{
    Filler f{ Slicer{buildCubeSurfaceMesh(0.5) }.getMesh() };

    EXPECT_EQ(18, f.getMeshFilling().countTriangles());
}

TEST_F(FillerTest, cube_stepSize0c5_no_slicing)
{
    Filler f{ toRelative(buildCubeSurfaceMesh(0.5)) };
    EXPECT_EQ(18, f.getMeshFilling().countTriangles());
}


/// Fill the interior of a 1x1x1 cube in a grid with step size = 0.25,
/// faces centered on the grid.
/// Expected result: 
/// -  30 = 5*2*3 triangles from the filled inside.
TEST_F(FillerTest, cube_stepSize0c25)
{
    Filler f{ Slicer{buildCubeSurfaceMesh(0.25) }.getMesh() };

    EXPECT_EQ(30, f.getMeshFilling().countTriangles());
}

/// Fill the interior of a 1x1x1 cube in a grid with step size = 1,
/// faces *not* centered on the grid.
TEST_F(FillerTest, cube_stepSize1_not_face_centered)
{
    auto m{ Slicer{buildCubeSurfaceMesh(1.0)}.getMesh() };
    addOffset(m.coordinates, Coordinate({0.5, 0.5, 0.5}));

    Filler f{m};

    EXPECT_EQ(6, f.getMeshFilling().countTriangles());
}

/// Fill the interior of a parallelogram that spans one cell on the xy plane 
/// and one cell and a half on the z direction. 
TEST_F(FillerTest, parallelogram_as_volume) 
{   
    Filler f{ Slicer{ buildParalelogramMesh() }.getMesh() };

    EXPECT_EQ(12, f.getMeshFilling().countTriangles());

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), X })));

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Z })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), Z })));
}

TEST_F(FillerTest, parallelogram_as_volume_no_slicing)
{
    Filler f{ toRelative(buildParalelogramMesh()) };

    EXPECT_EQ(12, f.getMeshFilling().countTriangles());

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), X })));

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Z })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), Z })));
}


TEST_F(FillerTest, parallelogram_as_surface)
{
    Filler f{ Mesh(), Slicer{ buildParalelogramMesh() }.getMesh() };

    EXPECT_EQ(10, f.getMeshFilling().countTriangles());

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), X })));
    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), X })));

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({0, 0, 0}), Z })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ Cell({0, 0, 1}), Z })));
}

TEST_F(FillerTest, planeXY_mesh_filling)
{
    Filler f{ Slicer{ buildPlaneXYMesh(1.0) }.getMesh() };

    auto mF{ f.getMeshFilling() };
    
    EXPECT_EQ(2, mF.countTriangles());
    EXPECT_EQ(0, mF.countLines());
}

TEST_F(FillerTest, planeXY_mesh_filling_2)
{
    auto m{ buildPlaneXYMesh(1.0) };
    std::transform(
        m.groups[0].elements.begin(), m.groups[0].elements.end(),
        m.groups[0].elements.begin(),
        [](auto& e) { 
            std::swap(e.vertices[0], e.vertices[1]);
            return e;
        }
    );
    Filler f{ Slicer{ m }.getMesh() };

    auto mF{ f.getMeshFilling() };

    EXPECT_EQ(2, mF.countTriangles());
    EXPECT_EQ(0, mF.countLines());
}

TEST_F(FillerTest, planeXY_mesh_filling_3)
{
    auto m{ buildSelfOverlappingMesh(1.0) };
    m.groups[0].elements[1] = Element({ 2, 1, 3 });
    m.coordinates[3] = Coordinate({0.75, 0.75, 0.00});

    auto mF{ Filler{ Slicer{ m }.getMesh() }.getMeshFilling() };

    EXPECT_EQ(2, mF.countTriangles());
    EXPECT_EQ(0, mF.countLines());
}

TEST_F(FillerTest, planeXY_stepSize0c5_mesh_filling)
{
    Filler f{ Slicer{ buildPlaneXYMesh(0.5) }.getMesh() };

    auto mF{ f.getMeshFilling() };

    EXPECT_EQ(2, mF.countTriangles());
    EXPECT_EQ(0, mF.countLines());
}

TEST_F(FillerTest, planeXY_face_filling)
{
    Filler f{ Slicer{ buildPlaneXYMesh(1.0) }.getMesh() };

    Cell c({ 1, 1, 1 });
    {
        auto fF{ f.getFaceFilling({ c, X }) };
        EXPECT_EQ(0, countPWHs(fF));
        EXPECT_EQ(0, countLins(fF));
    }
    {
        auto fF{ f.getFaceFilling({ c, Y }) };
        EXPECT_EQ(0, countPWHs(fF));
        EXPECT_EQ(0, countLins(fF));
    }
    {
        auto fF{ f.getFaceFilling({ c, Z }) };
        EXPECT_EQ(1, countPWHs(fF));
        EXPECT_EQ(0, countLins(fF));
    }
}

TEST_F(FillerTest, planeXY_edge_filling)
{
    Filler f{ Slicer{ buildPlaneXYMesh(1.0) }.getMesh() };

    {
        Cell c({ 1, 1, 1 });
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
    {
        Cell c({ 1, 1, 2 });
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
}

TEST_F(FillerTest, planeXY_edge_filling_empty_adjacent_cells)
{
    Filler f{ Slicer{ buildPlaneXYMesh(1.0) }.getMesh() };

    {
        Cell c({ 2, 1, 1 });
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
    {
        Cell c({ 2, 2, 1 });
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
    {
        Cell c({ 0, 1, 1 });
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
    {
        Cell c({ 0, 2, 1 });
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Y })));
        EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
    }
}

TEST_F(FillerTest, frameXY_stepSize1_mesh_filling)
{
    Filler f{ Slicer{ buildFrameXYMesh(1.0) }.getMesh() };

    EXPECT_EQ(8, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());
}

TEST_F(FillerTest, frameXY_stepSize1_face_filling)
{
    Filler f{ Slicer{ buildFrameXYMesh(1.0) }.getMesh() };

    auto fF{ f.getFaceFilling({ Cell({ 0, 0, 0 }), Z }) };
    EXPECT_EQ(1, countPWHs(fF));
    EXPECT_EQ(0, countLins(fF));
}

TEST_F(FillerTest, frameXY_stepSize0c25_face_filling)
{
    Filler f{ Slicer{ buildFrameXYMesh(0.25) }.getMesh() };

    EXPECT_EQ(1, countPWHs(f.getFaceFilling({ Cell({ 0, 0, 0 }), Z })));
    EXPECT_EQ(0, countPWHs(f.getFaceFilling({ Cell({ 1, 1, 0 }), Z })));
}

TEST_F(FillerTest, frameXY_same_result_for_volume_mesh_and_surface_mesh)
{
    auto m{ Slicer{ buildFrameXYMesh(0.25)}.getMesh() };
    Filler fv{ m };
    Filler fs{ Mesh(), m };
    
    EXPECT_EQ(fv.getMeshFilling(), fs.getMeshFilling());
    
    {
        CellIndex c{ Cell({0,0,0}), Z };
        EXPECT_EQ(fs.getFaceFilling(c), fv.getFaceFilling(c));
        EXPECT_EQ(fs.getEdgeFilling(c), fv.getEdgeFilling(c));
    }
}

TEST_F(FillerTest, frameXY_stepSize1_edge_filling)
{
    Filler f{ Slicer{ buildFrameXYMesh(1.0) }.getMesh() };

    EXPECT_EQ(1, countLins(f.getEdgeFilling({ Cell({0,0,0}), X })));
    EXPECT_EQ(1, countLins(f.getEdgeFilling({ Cell({0,0,0}), Y })));
    EXPECT_EQ(0, countLins(f.getEdgeFilling({ Cell({0,0,0}), Z })));

    EXPECT_EQ(0, countLins(f.getEdgeFilling({ Cell({1,0,0}), X })));
    EXPECT_EQ(1, countLins(f.getEdgeFilling({ Cell({1,0,0}), Y })));
    
    EXPECT_EQ(1, countLins(f.getEdgeFilling({ Cell({0,1,0}), X })));
    EXPECT_EQ(0, countLins(f.getEdgeFilling({ Cell({0,1,0}), Y })));
}

TEST_F(FillerTest, planeXY_notFaceCentered_mesh_filling)
{
    auto m{ buildPlaneXYMesh(1.0) };
    addOffset(m.coordinates, Coordinate({ 0.0, 0.0, 0.5 }));

    Filler f{ Slicer{ m }.getMesh() };

    EXPECT_EQ(0, f.getMeshFilling().countTriangles());
    EXPECT_EQ(4, f.getMeshFilling().countLines());
}

TEST_F(FillerTest, planeXY_notFaceCentered_face_filling)
{
    auto m{ buildPlaneXYMesh(1.0) };
    addOffset(m.coordinates, Coordinate({ 0.0, 0.0, 0.5 }));

    Filler f{ Slicer{ m }.getMesh() };

    Cell c({ 1, 1, 1 });
    {
        auto fF{ f.getFaceFilling({ c, X }) };
        EXPECT_EQ(0, countPWHs(fF));
        EXPECT_EQ(1, countLins(fF));
    }
    {
        auto fF{ f.getFaceFilling({ c, Y }) };
        EXPECT_EQ(0, countPWHs(fF));
        EXPECT_EQ(1, countLins(fF));
    }
    {
        auto fF{ f.getFaceFilling({ c, Z }) };
        EXPECT_EQ(0, countPWHs(fF));
        EXPECT_EQ(0, countLins(fF));
    }
}

TEST_F(FillerTest, planeXY_notFaceCentered_edge_filling)
{
    auto m{ buildPlaneXYMesh(1.0) };
    addOffset(m.coordinates, Coordinate({ 0.0, 0.0, 0.5 }));

    Filler f{ Slicer{ m }.getMesh() };

    Cell c({ 1, 1, 1 });
    EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, X })));
    EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Y })));
    EXPECT_EQ(0, countLins(f.getEdgeFilling({ c, Z })));
}

TEST_F(FillerTest, two_planes_on_same_face)
{
    Filler f{ buildTwoSquaresXYMesh(1.0) };

    EXPECT_EQ(4, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());

    Cell c({ 0, 0, 0 });
    EXPECT_EQ(2, countPWHs(f.getFaceFilling({ c, Z })));
}

TEST_F(FillerTest, two_planes_on_same_face_different_pr)
{
    Filler f{ buildTwoSquaresTwoGroupsXYMesh(1.0), Mesh{}, std::vector<Priority>{10,0} };

    EXPECT_EQ(4, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());

    {
        Cell c({ 0, 0, 0 });
        auto ff{ f.getFaceFilling({ c, Z }) };

        EXPECT_EQ(2, countPWHs(ff));

        ASSERT_TRUE(ff.tris.find(10) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[10].size());
        ASSERT_TRUE(ff.tris.find(0) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[0].size());
        
        auto ef{ f.getEdgeFilling({ c, X }) };

        EXPECT_EQ(1, countLins(ef));

        ASSERT_TRUE(ef.lins.find(10) != ef.lins.end());
        EXPECT_EQ(1, ef.lins[10].size());
        ASSERT_TRUE(ef.lins.find(0) == ef.lins.end());

    }
    {
        Cell c({ 0, 1, 0 });
        auto ef{ f.getEdgeFilling({ c, X }) };
        EXPECT_EQ(1, countLins(ef));
        ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
        EXPECT_EQ(1, ef.lins[0].size());
        ASSERT_TRUE(ef.lins.find(10) == ef.lins.end());

    }
}

TEST_F(FillerTest, two_overlapping_groups_different_pr_not_simple)
{
    const Priority pr1{ 1 }, pr2{ 10 };
    Filler f{ 
        buildOverlappingTwoGroupsXYMeshNotSimple(1.0), 
        Mesh{}, 
        std::vector<Priority>{pr1, pr2} 
    };

    EXPECT_EQ(4, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());

    auto ff{ f.getFaceFilling({ Cell({ 0,0,0 }) , Z})};
    ASSERT_TRUE(ff.tris.find(pr1) != ff.tris.end());
    EXPECT_EQ(2, ff.tris[pr1].size());
    ASSERT_TRUE(ff.tris.find(pr2) != ff.tris.end());
    EXPECT_EQ(2, ff.tris[pr2].size());

    EXPECT_TRUE(allAreSimple(ff));
}

TEST_F(FillerTest, two_overlapping_groups_different_pr_on_same_face)
{
    Filler f{ buildOverlappingTwoGroupsXYMesh(1.0), Mesh{}, std::vector<Priority>{10,0} };

    EXPECT_EQ(6, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());

    {
        Cell c({ 0, 0, 0 });
        auto ff{ f.getFaceFilling({ c, Z }) };
        ASSERT_TRUE(ff.tris.find(10) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[10].size());
        ASSERT_TRUE(ff.tris.find(0) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[0].size());

        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));

    }
    {
        Cell c({ 1, 0, 0 });
        auto ef{ f.getEdgeFilling({ c, Y }) };
        ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
        EXPECT_EQ(1, ef.lins[0].size());
        ASSERT_TRUE(ef.lins.find(10) == ef.lins.end());

        EXPECT_EQ(1, countLins(ef));
    }
    {
        Cell c({ 0, 1, 0 });
        auto ef{ f.getEdgeFilling({ c, X }) };
        ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
        EXPECT_EQ(1, ef.lins[0].size());
        ASSERT_TRUE(ef.lins.find(10) == ef.lins.end());

        EXPECT_EQ(1, countLins(ef));
    }
}

TEST_F(FillerTest, two_overlapping_groups_different_pr_on_same_face_2)
{
    Priority pr1{ -10 }, pr2{ 20 };
    Filler f{ buildOverlappingTwoGroupsXYMesh(1.0), Mesh{}, std::vector<Priority>{pr1,pr2} };

    EXPECT_EQ(6, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());

    {
        Cell c({ 0, 0, 0 });
        auto ff{ f.getFaceFilling({ c, Z }) };
        ASSERT_TRUE(ff.tris.find(pr1) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[pr1].size());
        ASSERT_TRUE(ff.tris.find(pr2) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[pr2].size());
        EXPECT_FLOAT_EQ(0.32, (float) ff.tris[pr1].area());
        EXPECT_FLOAT_EQ(0.36, (float) ff.tris[pr2].area());

        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, f.getEdgeFilling({ c, X }).lins.count(pr1));
        EXPECT_EQ(0, f.getEdgeFilling({ c, X }).lins.count(pr2));

        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
    }
    {
        Cell c({ 1, 0, 0 });
        auto ef{ f.getEdgeFilling({ c, Y }) };
        EXPECT_EQ(0, ef.lins.count(pr1));
        EXPECT_EQ(1, ef.lins.count(pr2));
    }
    {
        Cell c({ 0, 1, 0 });
        auto ef{ f.getEdgeFilling({ c, X }) };
        EXPECT_EQ(0, ef.lins.count(pr1));
        EXPECT_EQ(1, ef.lins.count(pr2));
    }

}

TEST_F(FillerTest, two_overlapping_groups_different_pr_on_same_edge)
{
    Filler f{ buildOverlappingTwoGroupsXYMesh2(1.0), Mesh{}, std::vector<Priority>{10,0} };

    {
        Cell c({ 0, 0, 0 });
        auto ff{ f.getFaceFilling({ c, Z }) };

        EXPECT_EQ(2, countPWHs(ff));

        ASSERT_TRUE(ff.tris.find(10) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[10].size());
        ASSERT_TRUE(ff.tris.find(0) != ff.tris.end());
        EXPECT_EQ(1, ff.tris[0].size());

        auto ef{ f.getEdgeFilling({ c, X }) };
        EXPECT_EQ(2, countLins(ef));
        ASSERT_TRUE(ef.lins.find(10) != ef.lins.end());
        ASSERT_TRUE(ef.lins.find(0)  != ef.lins.end());
        EXPECT_EQ(1, ef.lins[10].size());
        EXPECT_EQ(1, ef.lins[0].size());
        EXPECT_FLOAT_EQ((float) ef.lins[0][0][0], 0.6);

        ef = f.getEdgeFilling({ c, Y });
        EXPECT_EQ(1, countLins(ef));
        ASSERT_TRUE(ef.lins.find(10) != ef.lins.end());
        ASSERT_TRUE(ef.lins.find(0)  == ef.lins.end());
        EXPECT_EQ(1, ef.lins[10].size());


    }
    {
        Cell c({ 1, 0, 0 });
        auto ef{ f.getEdgeFilling({ c, Y }) };
        ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
        EXPECT_EQ(1, ef.lins[0].size());
        ASSERT_TRUE(ef.lins.find(10) == ef.lins.end());

        EXPECT_EQ(1, countLins(ef));
    }
    {
        Cell c({ 0, 1, 0 });
        auto ef{ f.getEdgeFilling({ c, X }) };
        EXPECT_EQ(0, countLins(ef));
    }

}

TEST_F(FillerTest, triangle_on_XZ_as_volume)
{
    Filler f{ buildOneTriangleOnXZ(1.0) };

    Cell c({ 0, 0, 0 });
    auto ff{ f.getFaceFilling({ c, Y }) };

    EXPECT_EQ(1, countPWHs(ff));

    ASSERT_TRUE(ff.tris.find(0) != ff.tris.end());
    EXPECT_EQ(1, ff.tris[0].size());

    auto ef{ f.getEdgeFilling({ c, X }) };
    EXPECT_EQ(1, countLins(ef));
    ASSERT_TRUE(ef.lins.find(0)  != ef.lins.end());
    EXPECT_EQ(1, ef.lins[0].size());

    ef = f.getEdgeFilling({ c, Z });
    EXPECT_EQ(1, countLins(ef));
    ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
    EXPECT_EQ(1, ef.lins[0].size());
}

TEST_F(FillerTest, triangle_on_XZ_as_surface)
{
    Filler f{ Mesh(), buildOneTriangleOnXZ(1.0) };

    Cell c({ 0, 0, 0 });
    auto ff{ f.getFaceFilling({ c, Y }) };

    EXPECT_EQ(1, countPWHs(ff));

    ASSERT_TRUE(ff.tris.find(0) != ff.tris.end());
    EXPECT_EQ(1, ff.tris[0].size());

    auto ef{ f.getEdgeFilling({ c, X }) };
    EXPECT_EQ(1, countLins(ef));
    ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
    EXPECT_EQ(1, ef.lins[0].size());

    ef = f.getEdgeFilling({ c, Z });
    EXPECT_EQ(1, countLins(ef));
    ASSERT_TRUE(ef.lins.find(0) != ef.lins.end());
    EXPECT_EQ(1, ef.lins[0].size());

}


TEST_F(FillerTest, two_overlapping_groups_same_pr_on_same_face)
{
    Filler f{ buildOverlappingTwoGroupsXYMesh(1.0), Mesh{}, std::vector<Priority>{0,0} };

    EXPECT_EQ(6, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());
    {
        Cell c({ 0, 0, 0 });
        EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, Z })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
    }
    {
        Cell c({ 1, 0, 0 });
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
    }
    {
        Cell c({ 0, 1, 0 });
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
    }
}

TEST_F(FillerTest, two_overlapping_groups_same_pr_on_same_face_2)
{
    Filler f{ buildOverlappingTwoGroupsXYMesh2(1.0), Mesh{}, std::vector<Priority>{0,0} };
    EXPECT_EQ(2, f.getMeshFilling().countTriangles());
    EXPECT_EQ(0, f.getMeshFilling().countLines());
    {
        Cell c({ 0, 0, 0 });
        EXPECT_EQ(1, countPWHs(f.getFaceFilling({ c, Z })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, X })));
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
    }
    {
        Cell c({ 1, 0, 0 });
        EXPECT_EQ(1, countLins(f.getEdgeFilling({ c, Y })));
    }
}

TEST_F(FillerTest, selfOverlapping_stepSize1)
{
    EXPECT_EQ(1, Filler{ buildSelfOverlappingMesh(1.0) }.getMeshFilling().countTriangles());
}

TEST_F(FillerTest, selfOverlapping_partially_stepSize1)
{
    auto m{ buildSelfOverlappingMesh(1.0) };
    m.coordinates[3] = Coordinate({ 0.9, 0.9, 0.0 });
    
    EXPECT_EQ(3, Filler{ m }.getMeshFilling().countTriangles());
}

TEST_F(FillerTest, fillingDetection_tet_surface)
{
    Filler f{ Slicer{buildTetSurfaceMesh(0.5)}.getMesh() };
    
    EXPECT_EQ(FillingType::Full,    f.getFillingState({ Cell({0, 0, 0}), Z }).type);
    EXPECT_EQ(FillingType::Partial, f.getFillingState({ Cell({1, 0, 0}), Z }).type);
    EXPECT_EQ(FillingType::Empty,   f.getFillingState({ Cell({1, 1, 0}), Z }).type);
}

TEST_F(FillerTest, fillingDetection_cube)
{
    auto m{ Slicer{ buildCubeSurfaceMesh(0.5) }.getMesh() };
    Filler f{ m };
    for (auto i{ 0 }; i < m.grid[X].size(); ++i) {
        for (auto j{ 0 }; j < m.grid[Y].size(); ++j) {
            for (auto k{ 0 }; k < m.grid[Z].size(); ++k) {
                Cell c({ i, j, k });
                for (const auto& x : { X, Y, Z }) {
                    CellIndex cI{c , x};
                    auto state{ f.getFillingState(cI) };
                    EXPECT_TRUE(state.empty() || state.full());
                }
            }
        }
    }
}

