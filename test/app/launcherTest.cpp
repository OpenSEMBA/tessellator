#include <gtest/gtest.h>

#include "app/launcher.h"
#include "types/Mesh.h"
#include <nlohmann/json.hpp>
#include <fstream>

using namespace meshlib::app;

class LauncherTest : public ::testing::Test
{
};

TEST_F(LauncherTest, prints_help)
{
    int ac = 2;
    const char* av[] = { NULL, "-h" };
    EXPECT_EQ(meshlib::app::launcher(ac, av), EXIT_SUCCESS);
}

TEST_F(LauncherTest, parse_rectilinear_grid)
{
    int ac = 3;
    std::string fileName = "testData/cases/longPolyline/RectilinearGrid.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }

    meshlib::Grid grid = meshlib::app::parseGridFromJSON(j["grid"]);

    meshlib::Grid expectedGrid({
        std::vector<double>{600, 603.25},
        std::vector<double>{25.0, 30.5, 92, 130, 1000},
        std::vector<double>{1000, 1111, 1111.1}
    });

    for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto& planes = grid[axis];
        const auto& expectedPlanes = expectedGrid[axis];

        EXPECT_EQ(planes.size(), expectedPlanes.size());


        for (std::size_t planeIndex = 0; planeIndex < grid[axis].size(); ++planeIndex) {
            EXPECT_EQ(planes[planeIndex], expectedPlanes[planeIndex]);
        }
    }
}

TEST_F(LauncherTest, launches_alhambra_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/alhambra/alhambra.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_alhambra_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/alhambra/alhambra.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}


TEST_F(LauncherTest, launches_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/sphere.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_closed_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/closed_sphere.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/sphere.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_thinCylinder_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/thinCylinder/thinCylinder.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_thinCylinder_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/thinCylinder/thinCylinder.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_cone_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/cone/cone.tessellator.json" };
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_long_polyline_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/longPolyline/longPolyline.tessellator.json" };
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_cone_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/cone/cone.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

