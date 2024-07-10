#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "converter/ConverterTools.h"

#include "Driver.h"
#include "utils/Geometry.h"

using namespace meshlib;
using namespace tessellator;
using namespace filler;
using namespace meshFixtures;

class DriverTest : public ::testing::Test {
public:
    static DriverOptions buildSnappedOptions() {
        DriverOptions opts;
        opts.collapseInternalPoints = true;
        opts.snap = true;
        return opts;
    }

    static DriverOptions buildAdaptedOptions() {
        DriverOptions opts;
        opts.collapseInternalPoints = true;
        opts.snap = false;
        return opts;
    }

    static DriverOptions buildRawOptions() {
        DriverOptions opts;
        opts.forceSlicing = true;
        opts.collapseInternalPoints = false;
        opts.snap = false;
        return opts;
    }

    static DriverOptions buildBareOptions() {
        DriverOptions opts;
        opts.forceSlicing = false;
        opts.collapseInternalPoints = false;
        opts.snap = false;
        return opts;
    }

    static auto getBoundingBoxOfUsedCoordinates(const Mesh& msh) {
        Coordinate min(std::numeric_limits<double>::max());
        Coordinate max(std::numeric_limits<double>::min());
        for (auto const& g : msh.groups) {
            for (auto const& e : g.elements) {
                for (auto const& vId : e.vertices) {
                    auto const& c = msh.coordinates[vId];
                    if (c < min) {
                        min = c;
                    }
                    if (c > max) {
                        max = c;
                    }
                }
            }
        }
        return std::make_pair(min, max);
    }

    static auto buildFittedGrid(const std::pair<Coordinate, Coordinate>& bb, double stepSize)
    {
        auto const& boxMin = bb.first;
        auto const& boxMax = bb.second;
        Grid grid;
        for (std::size_t d = 0; d < 3; d++) {
            const std::size_t num = (std::size_t)((boxMax(d) - boxMin(d)) / (double)stepSize) + 1;
            grid[d] = utils::GridTools::linspace(boxMin(d), boxMax(d), num);
        }
        return grid;
    }

    static std::size_t countRepeatedElements(const Mesh& m) 
    {
        std::set<std::set<CoordinateId>> verticesSets;
        for (auto const& g : m.groups) {
            for (auto const& e : g.elements) {
                verticesSets.insert(std::set<CoordinateId>(e.vertices.begin(), e.vertices.end()));
            }
        }
        return m.countElems() - verticesSets.size();
    }

};

TEST_F(DriverTest, cubes_overlap_different_materials)
{
    auto opts{ buildAdaptedOptions() };
    opts.volumeGroups = { 0, 1 };

    auto r{ Driver{ buildTwoCubesWithOffsetMesh(1.0), opts }.mesh() };

    ASSERT_EQ(2, r.groups.size());

    EXPECT_EQ(12, r.groups[0].elements.size());
    EXPECT_EQ(20, r.groups[1].elements.size());
}

TEST_F(DriverTest, two_materials_with_overlap_no_intersect)
{
    auto opts = buildAdaptedOptions();
    
    Mesh outOneMat;
    {
        Mesh oneMat = buildTwoMaterialsMesh();
        oneMat.groups.pop_back();
        Driver mesher(oneMat, opts);
        ASSERT_NO_THROW(outOneMat = mesher.mesh());
    }

    Mesh outTwoMats;
    {
        Mesh twoMats = buildTwoMaterialsMesh();
        Driver mesher(twoMats, opts);
        ASSERT_NO_THROW(outTwoMats = mesher.mesh());
    }
    
    ASSERT_EQ(1, outOneMat.groups.size());
    ASSERT_EQ(2, outTwoMats.groups.size());   
}

TEST_F(DriverTest, two_materials_with_no_overlap_same_as_one_material)
{
    auto opts = buildAdaptedOptions();
    double stepSize = 1.0;

    Mesh m = Driver(buildPlane45TwoMaterialsMesh(stepSize), opts).mesh();

    VecD n = utils::Geometry::normal(
        utils::Geometry::asTriV(m.groups[0].elements[0], m.coordinates)
    );
    for (auto const& g : m.groups) {
        for (auto const& e : g.elements) {
            TriV tri = utils::Geometry::asTriV(e, m.coordinates);
            VecD n2 = utils::Geometry::normal(tri);
            EXPECT_NEAR(0.0, n.angleDeg(n2), 1e-3);
        }
    }
}

TEST_F(DriverTest, plane45_size1_grid_adapted) 
{
    Driver mesher(buildPlane45Mesh(1.0), buildAdaptedOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(2, out.groups[0].elements.size());
}

TEST_F(DriverTest, plane45_size1_grid_raw)
{
    Driver mesher(buildPlane45Mesh(1.0), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(4, out.groups[0].elements.size());
}

TEST_F(DriverTest, tri_non_uniform_grid_adapted)
{
    Mesh out;
    ASSERT_NO_THROW(out = Driver(buildTriNonUniformGridMesh(), buildAdaptedOptions()).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(27, out.groups[0].elements.size());
}

TEST_F(DriverTest, tri_non_uniform_grid_snapped)
{
    auto opts = buildSnappedOptions();
    opts.snapperOptions.forbiddenLength = 0.25;

    Mesh out;
    ASSERT_NO_THROW(out = Driver(buildTriNonUniformGridMesh(), opts).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(24, out.groups[0].elements.size());
}

TEST_F(DriverTest, tet_size1_grid_raw) 
{
    Driver mesher(buildTetMesh(1.0), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(4, out.groups[0].elements.size());
}
TEST_F(DriverTest, tet_size1_grid_adapted) 
{
    Driver mesher(buildTetMesh(1.0), buildAdaptedOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(4, out.groups[0].elements.size());
}

TEST_F(DriverTest, bowtie_corner_size5_grid_raw)
{
    Driver mesher(buildCornerBowtieMesh(5.0), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(6, out.countTriangles());
}

TEST_F(DriverTest, bowtie_corner_size5_grid_adapted)
{

    Driver mesher(buildCornerBowtieMesh(5.0), buildAdaptedOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(6, out.countTriangles());
}

TEST_F(DriverTest, plane45_size05_grid_adapted) 
{
 
    Driver mesher(buildPlane45Mesh(0.5), buildAdaptedOptions());
    
    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(8, out.groups[0].elements.size());
}

TEST_F(DriverTest, tets_sharing_edge_adapted)
{
    Driver mesher(buildTetsSharingEdgeMesh(), buildAdaptedOptions());

    {
        Mesh out;
        ASSERT_NO_THROW(out = mesher.mesh());
    }
}

TEST_F(DriverTest, plane45_size05_grid_raw)
{

    Driver mesher(buildPlane45Mesh(0.5), buildRawOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(12, out.groups[0].elements.size());
}

TEST_F(DriverTest, plane45_size025_grid_adapted) {

    Driver mesher(buildPlane45Mesh(0.25), buildAdaptedOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(32, out.countTriangles());
}

TEST_F(DriverTest, plane45_size025_grid_raw) 
{

    Driver mesher(buildPlane45Mesh(0.25), buildRawOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(40, out.countTriangles());
}

TEST_F(DriverTest, bowtie_two_triangles_adapted)
{

    Driver mesher(buildTwoTrianglesFromBowtieCoarseMesh(), buildAdaptedOptions());
    
    Mesh out;
    Mesh dual;
    ASSERT_NO_THROW(out = mesher.mesh());
}

TEST_F(DriverTest, bowtie_subset_1_adapted)
{

    Driver mesher(buildBowtieSubset1Mesh(), buildAdaptedOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
}

TEST_F(DriverTest, tri) 
{

    Mesh in;
    {
        Grid grid;
        grid[0] = { 0.0, 1.0, 2.0 };
        grid[1] = { 0.0, 1.0, 2.0 };
        grid[2] = { 0.0, 1.0, 2.0 };

        std::vector<Coordinate> coords(3);
        coords[0] = Coordinate{ {0.1, 0.1, 0.1} };
        coords[1] = Coordinate{ {0.1, 1.9, 0.1} };
        coords[2] = Coordinate{ {0.9, 0.1, 0.1} };
        std::vector<Group> groups(1);
        groups[0] = { std::vector<Element>(1) };
        Element tri;
        tri.type = Element::Type::Surface;
        tri.vertices = { 0, 1, 2 };
        groups[0].elements[0] = tri;

        in = Mesh{ grid, coords, groups };
    }
    
    DriverOptions opts;
    opts.snap = false;

    Driver mesher(in, opts);

    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(3, out.countElems());

    Filler f{ mesher.fill() };
    EXPECT_EQ(0, f.getFaceFilling(CellIndex{ Cell{{0,0,0}}, Axis{Z} }).tris[0].size());
    EXPECT_EQ(1, f.getFaceFilling(CellIndex{ Cell{{0,1,0}}, Axis{Y} }).lins[0].size());


}

TEST_F(DriverTest, tri_dual)
{

    Mesh in;
    {
        Grid grid;
        grid[0] = { 0.0, 1.0 };
        grid[1] = { 0.0, 1.0 };
        grid[2] = { 0.0, 1.0 };

        std::vector<Coordinate> coords(3);
        coords[0] = Coordinate{ {0.0, 0.5, 0.0} };
        coords[1] = Coordinate{ {1.0, 0.5, 0.0} };
        coords[2] = Coordinate{ {0.0, 0.5, 1.0} };
        std::vector<Group> groups(1);
        groups[0].elements = {
            Element({ 0, 1, 2 })
        };
        in = Mesh{ grid, coords, groups };
    }

    DriverOptions opts;
    opts.snap = false;

    Driver mesher(in, opts);
    EXPECT_EQ(1, mesher.mesh().countElems());

    Filler d{ mesher.dualFill() };    
    EXPECT_EQ(1, d.getFaceFilling(CellIndex{ Cell{{0,1,0}}, Y }).tris[0].size());
    EXPECT_EQ(1, d.getFaceFilling(CellIndex{ Cell{{0,1,1}}, Y }).tris[0].size());
    EXPECT_EQ(1, d.getFaceFilling(CellIndex{ Cell{{1,1,0}}, Y }).tris[0].size());

    EXPECT_EQ(0, d.getEdgeFilling(CellIndex{ Cell{{0,1,0}}, X }).lins[0].size());
    EXPECT_EQ(1, d.getEdgeFilling(CellIndex{ Cell{{0,1,1}}, X }).lins[0].size());
    EXPECT_EQ(1, d.getEdgeFilling(CellIndex{ Cell{{1,1,0}}, Z }).lins[0].size());
}

TEST_F(DriverTest, smoother_generates_triangles_crossing_grid) 
{
    Mesh m;
    {
        std::size_t num = (std::size_t) ((2.0 / 0.1) + 1);
        m.grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
        m.grid[1] = m.grid[0];
        m.grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
    }
    m.coordinates = {
        Coordinate({-2.76443354e-01, -4.16628218e-01, +4.90860585e-01}),
        Coordinate({-3.56575014e-01, -3.50505719e-01, +4.89279192e-01}),
        Coordinate({-4.23042588e-01, -2.66523861e-01, +4.60887718e-01}),
        Coordinate({-3.21778239e-01, -3.82699314e-01, +5.73835267e-01}),
        Coordinate({-3.94255934e-01, -3.07509770e-01, +5.72539467e-01}),
        Coordinate({-4.44257764e-01, -2.29423276e-01, +5.73036985e-01}),
        Coordinate({-3.62463121e-01, -3.44413249e-01, +6.57794402e-01}),
        Coordinate({-4.19125800e-01, -2.72641822e-01, +6.60495891e-01})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({4, 6, 3}, Element::Type::Surface),
        Element({7, 4, 5}, Element::Type::Surface),
        Element({5, 4, 2}, Element::Type::Surface),
        Element({4, 3, 1}, Element::Type::Surface),
        Element({1, 3, 0}, Element::Type::Surface)
    };

    auto opts = buildAdaptedOptions();
        
    opts.decimalPlacesInCollapser = 0;
    ASSERT_NO_THROW(Driver(m, opts).mesh());
}

TEST_F(DriverTest, smoother_generates_triangles_crossing_grid_2)
{
    Mesh m;
    {
        std::size_t num = (std::size_t)((2.0 / 0.1) + 1);
        m.grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
        m.grid[1] = m.grid[0];
        m.grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
    }
    m.coordinates = {
        Coordinate({-2.76443354e-01, -4.16628218e-01, +4.90860585e-01}),
        Coordinate({-3.56575014e-01, -3.50505719e-01, +4.89279192e-01}),
        Coordinate({-4.23042588e-01, -2.66523861e-01, +4.60887718e-01}),
        Coordinate({-3.21778239e-01, -3.82699314e-01, +5.73835267e-01}),
        Coordinate({-3.94255934e-01, -3.07509770e-01, +5.72539467e-01}),
        Coordinate({-4.44257764e-01, -2.29423276e-01, +5.73036985e-01})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({5, 4, 2}, Element::Type::Surface),
        Element({4, 3, 1}, Element::Type::Surface),
        Element({1, 3, 0}, Element::Type::Surface)
    };

    auto opts = buildAdaptedOptions();

    opts.decimalPlacesInCollapser = 0;
    ASSERT_NO_THROW(Driver(m, opts).mesh());
}

TEST_F(DriverTest, smoother_generates_triangles_crossing_grid_3)
{
    Mesh m;
    {
        std::size_t num = (std::size_t)((2.0 / 0.1) + 1);
        m.grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
        m.grid[1] = m.grid[0];
        m.grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
    }
    m.coordinates = {
        Coordinate({-2.48528014e-01, +4.33859224e-01,  +4.56962783e-01}),
        Coordinate({-2.09286083e-01, +4.54091770e-01,  +5.60745399e-01}),
        Coordinate({-3.34692020e-01, +3.71458277e-01,  +4.51246177e-01}),
        Coordinate({-3.04723986e-01, +3.96413033e-01,  +5.57786308e-01})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({2, 0, 3}, Element::Type::Surface),
        Element({1, 3, 0}, Element::Type::Surface)
    };

    auto opts = buildAdaptedOptions();

    opts.decimalPlacesInCollapser = 2;
    ASSERT_NO_THROW(Driver(m, opts).mesh());
}

TEST_F(DriverTest, smoother_generates_triangles_crossing_grid_4)
{

    auto opts = buildAdaptedOptions();
    opts.decimalPlacesInCollapser = 2;
    
    ASSERT_NO_THROW(Driver(buildCylinderPatchMesh(), opts).mesh());
}

TEST_F(DriverTest, smoother_generates_triangles_crossing_grid_5)
{
    Mesh m;
    {
        std::size_t num = (std::size_t)((2.0 / 0.1) + 1);
        m.grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
        m.grid[1] = m.grid[0];
        m.grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
    }
    m.coordinates = {
        Coordinate({-3.17006262e-01, -3.86661389e-01, +4.03433688e-01}),
        Coordinate({-2.76443354e-01, -4.16628218e-01, +4.90860585e-01}),
        Coordinate({-3.56575014e-01, -3.50505719e-01, +4.89279192e-01}),
        Coordinate({-3.21778239e-01, -3.82699314e-01, +5.73835267e-01})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({2, 3, 1}, Element::Type::Surface),
        Element({2, 1, 0}, Element::Type::Surface)
    };

    auto opts = buildAdaptedOptions();
    opts.decimalPlacesInCollapser = 1;

    ASSERT_NO_THROW(Driver(m, opts).mesh());
}

TEST_F(DriverTest, cube_1x1x1_volume_size_0c25_grid_raw)
{
    Driver mesher(buildCubeVolumeMesh(0.25), buildRawOptions());

    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    
    EXPECT_EQ(192, out.countTriangles());
}

TEST_F(DriverTest, cube_1x1x1_volume_size_0c5_grid_raw)
{

    Driver mesher(buildCubeVolumeMesh(0.5), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(48, out.countTriangles());

}

TEST_F(DriverTest, cube_1x1x1_volume_size_1c0_grid_raw)
{
    Mesh out;
    ASSERT_NO_THROW(out = Driver(buildCubeVolumeMesh(1.0), buildRawOptions()).mesh());
    
    EXPECT_EQ(12, out.countTriangles());
}

TEST_F(DriverTest, cube_1x1x1_volume_size_1c0_grid_snapped)
{
    Mesh p;

    Driver mesher(buildCubeVolumeMesh(1.0), buildSnappedOptions());
    ASSERT_NO_THROW(p = mesher.mesh());

    EXPECT_EQ(12, p.countTriangles());
}

TEST_F(DriverTest, cube_1x1x1_surface_treat_as_volume)
{
    Mesh p;

    auto opts{ buildSnappedOptions()};
    opts.volumeGroups = { 0 };

    Driver mesher(buildCubeSurfaceMesh(1.0), opts);
    ASSERT_NO_THROW(p = mesher.mesh());
    
    EXPECT_EQ(12, p.countTriangles());
}

TEST_F(DriverTest, slab_surface_treat_as_volume)
{

    auto opts{ buildSnappedOptions() };
    opts.snapperOptions.forbiddenLength = 0.25;
    opts.volumeGroups = { 0 };
    
    Mesh p;
    ASSERT_NO_THROW(p = Driver(buildSlabSurfaceMesh(1.0, 0.01), opts).mesh());
    
    EXPECT_EQ(4, p.countTriangles());
}

TEST_F(DriverTest, plane45_size1_grid)
{
    DriverOptions vOpts;
    vOpts.collapseInternalPoints = false;
    vOpts.snap = false;

    Mesh vMsh;
    ASSERT_NO_THROW(vMsh = Driver(buildPlane45Mesh(1.0), vOpts).mesh());
}

TEST_F(DriverTest, tet_size1_grid)
{
    Driver mesher(buildTetMesh(1.0), DriverOptions ());

    Mesh msh;
    ASSERT_NO_THROW(msh = mesher.mesh());

    EXPECT_EQ(4, msh.countTriangles());
}

TEST_F(DriverTest, tet_with_inner_point_size1_grid)
{
    Driver mesher(buildTetMeshWithInnerPoint(1.0), DriverOptions());

    Mesh msh;
    ASSERT_NO_THROW(msh = mesher.mesh());

    EXPECT_EQ(4, msh.countTriangles());
}

TEST_F(DriverTest, elementsPartiallyOutOfGrid_bare)
{
    Driver h{ buildTriPartiallyOutOfGridMesh(0.5), buildBareOptions() };
     
    EXPECT_EQ(2, h.mesh().countTriangles());

    auto f{ h.fill() };
    EXPECT_EQ(2, f.getMeshFilling().countTriangles());
    EXPECT_TRUE(f.getFillingState({ Cell({ 0, 0, 1 }), Z }).partial());
    EXPECT_TRUE(f.getFillingState({ Cell({ 1, 1, 1 }), Z }).full());
    EXPECT_TRUE(f.getFillingState({ Cell({ 2, 2, 1 }), Z }).full());
    EXPECT_TRUE(f.getFillingState({ Cell({ 3, 3, 1 }), Z }).partial());
}

TEST_F(DriverTest, elementsPartiallyOutOfGrid_raw)
{
    Driver h{ buildTriPartiallyOutOfGridMesh(0.5), buildRawOptions() };

    EXPECT_EQ(32, h.mesh().countTriangles());

    auto f{ h.fill() };
    EXPECT_TRUE(f.getFillingState({ Cell({ 0, 0, 1 }), Z }).partial());
    EXPECT_TRUE(f.getFillingState({ Cell({ 1, 1, 1 }), Z }).full());
    EXPECT_TRUE(f.getFillingState({ Cell({ 2, 2, 1 }), Z }).full());
    EXPECT_TRUE(f.getFillingState({ Cell({ 3, 3, 1 }), Z }).partial());
}

TEST_F(DriverTest, elementsPartiallyOutOfGrid_dual_bare)
{
    Driver h{ buildTriPartiallyOutOfGridMesh(1.0), buildBareOptions() };

    EXPECT_EQ(2, h.mesh().countTriangles());

    auto df{ h.dualFill() };
    EXPECT_TRUE(df.getFillingState({ Cell({ 0, 0, 1 }), Z }).partial());
    EXPECT_TRUE(df.getFillingState({ Cell({ 1, 0, 1 }), Z }).partial());
    EXPECT_TRUE(df.getFillingState({ Cell({ 1, 1, 1 }), Z }).full());
    EXPECT_TRUE(df.getFillingState({ Cell({ 0, 1, 1 }), Z }).full());
}

TEST_F(DriverTest, elementsTotallyOutOfGrid)
{
    Mesh m = buildTriPartiallyOutOfGridMesh(1.0);
    m.coordinates = {
        Coordinate({100.0, 100.0,  100.0}),
        Coordinate({ 90.0,  90.0,   90.0}),
        Coordinate({ 70.0,  70.0,   70.0}),
    };

    Driver h{ m };

    EXPECT_EQ(0, h.mesh().countTriangles());

    EXPECT_EQ(0, h.fill().getMeshFilling().countElems());
    EXPECT_EQ(0, h.dualFill().getMeshFilling().countElems());
}

TEST_F(DriverTest, snapping_issue)
{
    Mesh m;
    m.grid =  buildProblematicTriMesh2().grid;
    m.coordinates = {
        Coordinate({-2.11727225e+01, +6.49226594e+00, +5.98379674e+00}),
        Coordinate({-2.11727225e+01, +7.59193327e+00, +5.98379674e+00}),
        Coordinate({-2.60623054e+01, +2.10000000e+01, +7.29395653e+00}),
    };
    m.groups = { Group{} };
    m.groups[0].elements = { {{0,1,2}} };

    DriverOptions opts;
    opts.snapperOptions.edgePoints = 0;
    opts.snapperOptions.forbiddenLength = 0.1;
    
    ASSERT_NO_THROW(Driver(m, opts).mesh());
}

TEST_F(DriverTest, meshed_with_holes_issue)
{
    Mesh m;
    m.grid = buildProblematicTriMesh2().grid;
    m.coordinates = {
        Coordinate({+2.24233492e+01, +3.10000000e+01, -5.18009738e+00}),
        Coordinate({+1.27640910e+01, +2.10000000e+01, -2.59190693e+00}),
        Coordinate({+1.27640910e+01, -9.00000000e+00, -2.59190693e+00}),
    };
    m.groups = { Group{} };
    m.groups[0].elements = { {{0,1,2}} };

    Mesh out;
    ASSERT_NO_THROW(out = Driver(m, buildRawOptions()).mesh());

    EXPECT_LT(374, out.countTriangles());
}