#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Snapper.h"
#include "Slicer.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace core;
using namespace utils;
using namespace meshFixtures;

class SnapperTest : public ::testing::Test {
protected:
};

TEST_F(SnapperTest, similar_results_for_each_plane)
{
    SnapperOptions opts;
    opts.edgePoints = 7;
    opts.forbiddenLength = 0.1;

    for (auto z: {0.0, 1.0}) {

        Mesh m;
        m.grid = buildUnitLengthGrid(1.0);
        m.coordinates = {
            Relative({0.80, 1.00, z}),
            Relative({1.00, 1.00, z}),
            Relative({0.80, 0.00, z}),
            Relative({1.00, 0.00, z})
        };
        m.groups = { Group() };
        m.groups[0].elements = {
            Element({0, 1, 2}),
            Element({2, 1, 3})
        };
                
        auto res = Snapper(m, opts).getMesh();
        EXPECT_EQ(res.coordinates, m.coordinates);

    }
}