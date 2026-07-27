#include "MeshFixtures.h"
#include "gtest/gtest.h"

#include "VolumeFiller.h"
#include "Slicer.h"
#include "Geometry.h"
#include "MeshTools.h"
#include "GridTools.h"
// #include "utils/RedundancyCleaner.h"
// #include "utils/CoordGraph.h"

#if APP_LOADED
    #include "app/vtkIO.h"
#endif

namespace meshlib::core {

using namespace meshFixtures;
using namespace utils;
using namespace meshTools;

class VolumeFillerTest : public ::testing::Test {
public:
protected:
};

static Mesh toRelative(const Mesh& m) 
{
    auto r{ m };
    r.coordinates = 
        utils::GridTools{ m.grid }.absoluteToRelative(m.coordinates);
    return r;
}

TEST_F(VolumeFillerTest, fill_cube1x1x1_size1_grid)
{
    Mesh m = buildCubeSurfaceMesh(01.0);
        
    Mesh out;
    ASSERT_NO_THROW(out = Slicer{m}.getMesh());
    EXPECT_EQ(12, countMeshElementsIf(out, isTriangle));

    Mesh filled = VolumeFiller{out}.getMesh();
    EXPECT_EQ(12, countMeshElementsIf(filled, isTriangle));
}

TEST_F(VolumeFillerTest, fill_cube1x1x1_size05_grid)
{
    //filling with unstruc. triangles
    Mesh m = buildCubeSurfaceMesh(0.5);
    Mesh filled = VolumeFiller{Slicer{buildCubeSurfaceMesh(0.5) }.getMesh()}.getMesh();
    Mesh filled_no_slicing = VolumeFiller{toRelative(buildCubeSurfaceMesh(0.5))}.getMesh();
    EXPECT_EQ(18, countMeshElementsIf(filled, isTriangle));
    EXPECT_EQ(18, countMeshElementsIf(filled_no_slicing, isTriangle));
        
    //slicing hull
    Mesh out_1;
    ASSERT_NO_THROW(out_1 = Slicer{buildCubeSurfaceMesh(0.5) }.getMesh());
    EXPECT_EQ(48, countMeshElementsIf(out_1, isTriangle));

    //filling and slicing
    Mesh out_2;
    GridTools gT{ filled.grid };
    filled.coordinates = gT.relativeToAbsolute(filled.coordinates);
    ASSERT_NO_THROW(out_2 = Slicer{filled }.getMesh());
    EXPECT_EQ(72, countMeshElementsIf(out_2, isTriangle));

    GridTools gT_no{ filled_no_slicing.grid };
    filled_no_slicing.coordinates = gT_no.relativeToAbsolute(filled_no_slicing.coordinates);
    ASSERT_NO_THROW(out_2 = Slicer{filled_no_slicing}.getMesh());
    EXPECT_EQ(72, countMeshElementsIf(out_2, isTriangle));

}

TEST_F(VolumeFillerTest, fill_cube1x1x1_size025_grid)
{
    Mesh m = buildCubeSurfaceMesh(0.25);
    Mesh filled = VolumeFiller{Slicer{buildCubeSurfaceMesh(0.25) }.getMesh()}.getMesh();
    EXPECT_EQ(30, countMeshElementsIf(filled, isTriangle));
        
    Mesh out_1;
    ASSERT_NO_THROW(out_1 = Slicer{buildCubeSurfaceMesh(0.25) }.getMesh());
    EXPECT_EQ(192, countMeshElementsIf(out_1, isTriangle));

    Mesh out_2;
    GridTools gT{ filled.grid };
    filled.coordinates = gT.relativeToAbsolute(filled.coordinates);

    ASSERT_NO_THROW(out_2 = Slicer{filled }.getMesh());
    EXPECT_EQ(480, countMeshElementsIf(out_2, isTriangle));

    // Mesh filled = VolumeFiller{out}.getMesh();
    // ASSERT_NO_THROW(out = Slicer{filled}.getMesh());
    // EXPECT_EQ(72, countMeshElementsIf(out, isTriangle));
}
}