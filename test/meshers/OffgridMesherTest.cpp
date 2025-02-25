#include "gtest/gtest.h"
#include "MeshFixtures.h"
#include "MeshTools.h"

#include "meshers/OffgridMesher.h"
#include "utils/Geometry.h"

namespace meshlib::meshers {
using namespace meshFixtures;
using namespace utils::meshTools;

class OffgridMesherTest : public ::testing::Test {
public:
    static OffgridMesherOptions buildSnappedOptions() {
        OffgridMesherOptions opts;
        opts.collapseInternalPoints = true;
        opts.snap = true;
        return opts;
    }

    static OffgridMesherOptions buildAdaptedOptions() {
        OffgridMesherOptions opts;
        opts.collapseInternalPoints = true;
        opts.snap = false;
        return opts;
    }

    static OffgridMesherOptions buildRawOptions() {
        OffgridMesherOptions opts;
        opts.forceSlicing = true;
        opts.collapseInternalPoints = false;
        opts.snap = false;
        return opts;
    }

    static OffgridMesherOptions buildBareOptions() {
        OffgridMesherOptions opts;
        opts.forceSlicing = false;
        opts.collapseInternalPoints = false;
        opts.snap = false;
        return opts;
    }

    static auto getBoundingBoxOfUsedCoordinates(const Mesh& msh) {
        Coordinate min(std::numeric_limits<double>::max());
        Coordinate max(std::numeric_limits<double>::lowest());
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

TEST_F(OffgridMesherTest, cubes_overlap_different_materials)
{
    auto opts{ buildAdaptedOptions() };
    opts.volumeGroups = { 0, 1 };

    auto r{ OffgridMesher{ buildTwoCubesWithOffsetMesh(1.0), opts }.mesh() };

    ASSERT_EQ(2, r.groups.size());

    EXPECT_EQ(12, r.groups[0].elements.size());
    EXPECT_EQ(20, r.groups[1].elements.size());
}

TEST_F(OffgridMesherTest, two_materials_with_overlap_no_intersect)
{
    auto opts = buildAdaptedOptions();
    
    Mesh outOneMat;
    {
        Mesh oneMat = buildTwoMaterialsMesh();
        oneMat.groups.pop_back();
        OffgridMesher mesher(oneMat, opts);
        ASSERT_NO_THROW(outOneMat = mesher.mesh());
    }

    Mesh outTwoMats;
    {
        Mesh twoMats = buildTwoMaterialsMesh();
        OffgridMesher mesher(twoMats, opts);
        ASSERT_NO_THROW(outTwoMats = mesher.mesh());
    }
    
    ASSERT_EQ(1, outOneMat.groups.size());
    ASSERT_EQ(2, outTwoMats.groups.size());   
}

TEST_F(OffgridMesherTest, two_materials_with_no_overlap_same_as_one_material)
{
    auto opts = buildAdaptedOptions();
    double stepSize = 1.0;

    Mesh m = OffgridMesher(buildPlane45TwoMaterialsMesh(stepSize), opts).mesh();

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

TEST_F(OffgridMesherTest, plane45_size1_grid_adapted) 
{
    OffgridMesher mesher(buildPlane45Mesh(1.0), buildAdaptedOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(2, out.groups[0].elements.size());
}

TEST_F(OffgridMesherTest, plane45_size1_grid_raw)
{
    OffgridMesher mesher(buildPlane45Mesh(1.0), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(4, out.groups[0].elements.size());
}

TEST_F(OffgridMesherTest, tri_non_uniform_grid_adapted)
{
    Mesh out;
    ASSERT_NO_THROW(out = OffgridMesher(buildTriNonUniformGridMesh(), buildAdaptedOptions()).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(27, out.groups[0].elements.size());
}

TEST_F(OffgridMesherTest, tri_non_uniform_grid_snapped)
{
    auto opts = buildSnappedOptions();
    opts.snapperOptions.forbiddenLength = 0.25;

    Mesh out;
    ASSERT_NO_THROW(out = OffgridMesher(buildTriNonUniformGridMesh(), opts).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(24, out.groups[0].elements.size());
}

TEST_F(OffgridMesherTest, bowtie_corner_size5_grid_raw)
{
    OffgridMesher mesher(buildCornerBowtieMesh(5.0), buildRawOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(6, countMeshElementsIf(out, isTriangle));
}

TEST_F(OffgridMesherTest, bowtie_corner_size5_grid_adapted)
{

    OffgridMesher mesher(buildCornerBowtieMesh(5.0), buildAdaptedOptions());
    Mesh out;
    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(6, countMeshElementsIf(out, isTriangle));
}

TEST_F(OffgridMesherTest, plane45_size05_grid_adapted) 
{
 
    OffgridMesher mesher(buildPlane45Mesh(0.5), buildAdaptedOptions());
    
    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(8, out.groups[0].elements.size());
}


TEST_F(OffgridMesherTest, plane45_size05_grid_raw)
{

    OffgridMesher mesher(buildPlane45Mesh(0.5), buildRawOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(12, out.groups[0].elements.size());
}

TEST_F(OffgridMesherTest, plane45_size025_grid_adapted) {

    OffgridMesher mesher(buildPlane45Mesh(0.25), buildAdaptedOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(32, countMeshElementsIf(out, isTriangle));
}

TEST_F(OffgridMesherTest, plane45_size025_grid_raw) 
{

    OffgridMesher mesher(buildPlane45Mesh(0.25), buildRawOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
    EXPECT_EQ(40, countMeshElementsIf(out, isTriangle));
}

TEST_F(OffgridMesherTest, bowtie_two_triangles_adapted)
{

    OffgridMesher mesher(buildTwoTrianglesFromBowtieCoarseMesh(), buildAdaptedOptions());
    
    Mesh out;
    Mesh dual;
    ASSERT_NO_THROW(out = mesher.mesh());
}

TEST_F(OffgridMesherTest, bowtie_subset_1_adapted)
{

    OffgridMesher mesher(buildBowtieSubset1Mesh(), buildAdaptedOptions());

    Mesh out;

    ASSERT_NO_THROW(out = mesher.mesh());
}


TEST_F(OffgridMesherTest, smoother_generates_triangles_crossing_grid) 
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
    ASSERT_NO_THROW(OffgridMesher(m, opts).mesh());
}

TEST_F(OffgridMesherTest, smoother_generates_triangles_crossing_grid_2)
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
    ASSERT_NO_THROW(OffgridMesher(m, opts).mesh());
}

TEST_F(OffgridMesherTest, smoother_generates_triangles_crossing_grid_3)
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
    ASSERT_NO_THROW(OffgridMesher(m, opts).mesh());
}

TEST_F(OffgridMesherTest, smoother_generates_triangles_crossing_grid_4)
{

    auto opts = buildAdaptedOptions();
    opts.decimalPlacesInCollapser = 2;
    
    ASSERT_NO_THROW(OffgridMesher(buildCylinderPatchMesh(), opts).mesh());
}

TEST_F(OffgridMesherTest, smoother_generates_triangles_crossing_grid_5)
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

    ASSERT_NO_THROW(OffgridMesher(m, opts).mesh());
}

TEST_F(OffgridMesherTest, cube_1x1x1_surface_treat_as_volume)
{
    Mesh p;

    auto opts{ buildSnappedOptions()};
    opts.volumeGroups = { 0 };

    OffgridMesher mesher(buildCubeSurfaceMesh(1.0), opts);
    ASSERT_NO_THROW(p = mesher.mesh());
    
    EXPECT_EQ(12, countMeshElementsIf(p, isTriangle));
}

TEST_F(OffgridMesherTest, slab_surface_treat_as_volume)
{

    auto opts{ buildSnappedOptions() };
    opts.snapperOptions.forbiddenLength = 0.25;
    opts.volumeGroups = { 0 };
    
    Mesh p;
    ASSERT_NO_THROW(p = OffgridMesher(buildSlabSurfaceMesh(1.0, 0.01), opts).mesh());
    
    EXPECT_EQ(4, countMeshElementsIf(p, isTriangle));
}

TEST_F(OffgridMesherTest, plane45_size1_grid)
{
    OffgridMesherOptions vOpts;
    vOpts.collapseInternalPoints = false;
    vOpts.snap = false;

    Mesh vMsh;
    ASSERT_NO_THROW(vMsh = OffgridMesher(buildPlane45Mesh(1.0), vOpts).mesh());
}

TEST_F(OffgridMesherTest, snapping_issue)
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

    OffgridMesherOptions opts;
    opts.snapperOptions.edgePoints = 0;
    opts.snapperOptions.forbiddenLength = 0.1;
    
    ASSERT_NO_THROW(OffgridMesher(m, opts).mesh());
}

TEST_F(OffgridMesherTest, meshed_with_holes_issue)
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
    ASSERT_NO_THROW(out = OffgridMesher(m, buildRawOptions()).mesh());

    EXPECT_LT(374, countMeshElementsIf(out, isTriangle));
}

}