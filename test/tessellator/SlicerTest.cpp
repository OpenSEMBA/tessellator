#include "MeshFixtures.h"
#include "gtest/gtest.h"

#include "Slicer.h"
#include "Geometry.h"
#include "MeshTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace meshFixtures;
using namespace utils;

class SlicerTest : public ::testing::Test {
public:
protected:
    const std::size_t X = 0;
    const std::size_t Y = 1;
    const std::size_t Z = 2;

    static bool containsDegenerateTriangles(const Mesh& out)
    {
        for (auto const& g : out.groups) {
            for (auto const& e : g.elements) {
                if (e.isTriangle() && 
                    Geometry::isDegenerate(Geometry::asTriV(e, out.coordinates))) {
                    return true;
                }
            }
        }
        return false;
    }
};



TEST_F(SlicerTest, buildTrianglesFromPath_1)
{
    Coordinates cs = {
        Coordinate({33.597101430000002, 0.0000000000000000,  2.0990656099999998}),
        Coordinate({34.000000000000000, 0.74057837000000004, 2.0000000000000000}),
        Coordinate({34.000000000000000, 0.74057837999999998, 2.0000000000000000}),
        Coordinate({33.000000000000000, 0.91539934000000001, 2.2458822600000001}),
        Coordinate({33.054659219999998, 1.0000000000000000,  2.2324425200000002}),
        Coordinate({33.000000000000000, 0.0000000000000000,  2.2458822600000001}),
        Coordinate({34.000000000000000, 1.0000000000000000,  2.0000000000000000})
    };
    CoordinateIds path{ 3, 5, 0, 1, 2, 6, 4 };

    for (const auto& t : Slicer::buildTrianglesFromPath(cs, path)) {
        EXPECT_NE(0.0, utils::Geometry::area(utils::Geometry::asTriV(t, cs)));
    }

}

TEST_F(SlicerTest, buildTrianglesFromPath_2)
{
    Coordinates cs{
        Coordinate({34.000000000000000, 0.74057837000000004, 2.0000000000000000}),
        Coordinate({34.000000000000000, 0.74057837999999998, 2.0000000000000000}),
        Coordinate({34.141133750000002, 1.0000000000000000,  1.9652977199999999}),
        Coordinate({34.000000000000000, 1.0000000000000000,  2.0000000000000000})
    };
    CoordinateIds path{ 3, 1, 0, 2};

    for (const auto& t : Slicer::buildTrianglesFromPath(cs, path)) {
        EXPECT_NE(0.0, utils::Geometry::area(utils::Geometry::asTriV(t, cs)));
    }

}

TEST_F(SlicerTest, tri45_size1_grid) 
{
    Mesh in = buildTri45Mesh(1.0);
    

    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(1, out.countTriangles());
}

TEST_F(SlicerTest, tri45_2_size1_grid)
{
    Mesh m;
        m.grid = utils::GridTools::buildCartesianGrid(0.0, 3.0, 4);

    m.coordinates = {
        Coordinate({ 3.00, 0.00, 0.50 }),
        Coordinate({ 0.00, 3.00, 1.00 }),
        Coordinate({ 0.00, 3.00, 0.00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(5, out.countTriangles());
}

TEST_F(SlicerTest, tri45_3_size1_grid)
{
    Mesh m;
    m.grid = {
        utils::GridTools::linspace(-61.0, 61.0, 123),
        utils::GridTools::linspace(-59.0, 59.0, 119),
        utils::GridTools::linspace(-11.0, 11.0, 23)
    };
    m.coordinates = {
        Coordinate({ 14.000000000000000, -13.000000000000000, 1.0000000000000000 }),
        Coordinate({ 14.000000000000000, -13.000000000000000, 0.0000000000000000 }),
        Coordinate({ 11.000000000000000, -10.000000000000000, 0.50000000000000000 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    EXPECT_EQ(5, out.countTriangles());
}

TEST_F(SlicerTest, tri45_size05_grid) 
{
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ buildTri45Mesh(0.5) }.getMesh());

    EXPECT_EQ(3, out.countTriangles());
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, tri45_size025_grid) 
{
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{ buildTri45Mesh(0.25) }.getMesh());

    ASSERT_EQ(1, out.groups.size());
    EXPECT_EQ(out.countTriangles(), out.groups[0].elements.size());
    EXPECT_EQ(10, out.countTriangles());
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size1_grid)
{
    Mesh in = buildCubeSurfaceMesh(1.0);
    

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(12, out.countTriangles());
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size05_grid)
{
    Mesh in = buildCubeSurfaceMesh(0.5);
    

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(48, out.countTriangles());
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cube1x1x1_size3_grid)
{
    Mesh in = buildCubeSurfaceMesh(3.0);
    

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(12, out.countTriangles());
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, tri_less45_size025_grid)
{
    Mesh in = buildTri45Mesh(0.25);
    in.coordinates[0] = { 1.45000000e+00, 1.00000000e+00, 1.4500000e+00 };

    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{in}.getMesh());

    EXPECT_EQ(11, out.countTriangles());
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, meshTrisOnGridBoundaries)
{
    Mesh m;
    m.grid = {
        std::vector<double>({0.0, 1.0, 2.0}),
        std::vector<double>({0.0, 1.0, 2.0}),
        std::vector<double>({0.0, 1.0, 2.0})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2}, Element::Type::Surface)
    };

    {
        m.coordinates = {
            Coordinate({0.0, 0.0, 0.0}),
            Coordinate({1.0, 0.0, 0.0}),
            Coordinate({0.0, 1.0, 0.0})
        };

        Mesh sliced = Slicer{m}.getMesh();

        EXPECT_EQ(1, sliced.countTriangles());
    }

    {
        m.coordinates = {
            Coordinate({2.0, 2.0, 2.0}),
            Coordinate({2.0, 1.0, 2.0}),
            Coordinate({2.0, 2.0, 1.0})
        };

        Mesh sliced = Slicer{m}.getMesh();

        EXPECT_EQ(1, sliced.countTriangles());
    }
}

TEST_F(SlicerTest, tri_degenerate) 
{
    Mesh m;
    m.grid = {
        utils::GridTools::linspace(-61.0, 61.0, 123),
        utils::GridTools::linspace(-59.0, 59.0, 119),
        utils::GridTools::linspace(-11.0, 11.0, 23)
    };
    m.coordinates = {
        Coordinate({ +8.00000000e+00, -7.00000000e+00, +1.00000000e+00 }),
        Coordinate({ +5.25538121e+00, -2.43936816e+00, +1.00000000e+00 }),
        Coordinate({ +2.00000000e+00, -1.00000000e+00, +1.00000000e+00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({2, 1, 0}, Element::Type::Surface)
    };
 
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());
    
    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cell_faces_are_crossed)
{
    Mesh m = buildProblematicTriMesh();
    
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}


TEST_F(SlicerTest, cell_faces_are_crossed_2)
{
    Mesh m = buildProblematicTriMesh2();
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}

TEST_F(SlicerTest, cell_faces_are_crossed_3)
{
    Mesh m;
    m.grid = buildProblematicTriMesh2().grid;
    m.coordinates = {
        Coordinate({ +2.56922991e+01, -2.52166072e+01, -6.57364794e+00 }),
        Coordinate({ +2.71235688e+01, -2.90000000e+01, -6.95715551e+00 }),
        Coordinate({ +2.59161616e+01, -2.90000000e+01, -6.63363171e+00 })
    };
    m.groups = { Group{} };
    m.groups[0].elements = {
        Element{ {0, 1, 2} }
    };
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());

    ASSERT_NO_THROW(meshTools::checkNoNullAreasExist(out));
    EXPECT_FALSE(containsDegenerateTriangles(out));
}