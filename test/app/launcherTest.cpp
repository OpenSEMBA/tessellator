#include <gtest/gtest.h>

#include "app/launcher.h"
#include "types/Mesh.h"
#include "meshers/StaircaseMesher.h"

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

TEST_F(LauncherTest, builds_staircased_mesher_default)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };

    ObjectDefinition objDef;
    auto mesher = meshlib::app::buildMesher(meshMock, "testData/cases/longPolyline/longPolyline_legacy.tessellator.json", objDef);

    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.isVolume, false);
    EXPECT_EQ(options.compress, false);
}

TEST_F(LauncherTest, builds_staircased_mesher_without_compression)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };

    ObjectDefinition objDef;
    auto mesher = meshlib::app::buildMesher(meshMock, "testData/cases/longPolyline/longPolyline.tessellator.json", objDef);

    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.isVolume, false);
    EXPECT_EQ(options.compress, false);
}

TEST_F(LauncherTest, builds_staircased_mesher_with_compression)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };

    ObjectDefinition objDef;
    auto mesher = meshlib::app::buildMesher(meshMock, "testData/cases/longPolyline/longPolyline_compression.tessellator.json", objDef);

    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.isVolume, false);
    EXPECT_EQ(options.compress, true);
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
    const char* av[] = { NULL, "-i", "testData/cases/longPolyline/longPolyline_legacy.tessellator.json" };
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

TEST_F(LauncherTest, readObjectsFromJSON_basic)
{
    auto objects = readObjectsFromJSON("testData/cases/multiObject/basic.tessellator.json");
    EXPECT_EQ(objects.size(), 2);
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_EQ(objects[0].group, "sphere_group");
    EXPECT_TRUE(objects[0].isVolume);
    EXPECT_FALSE(objects[0].mesherOverride.has_value());
    EXPECT_EQ(objects[1].filename, "cone.stl");
    EXPECT_EQ(objects[1].group, "cone_group");
    EXPECT_FALSE(objects[1].isVolume);
    EXPECT_FALSE(objects[1].mesherOverride.has_value());
}

TEST_F(LauncherTest, readObjectsFromJSON_mixedMesher)
{
    auto objects = readObjectsFromJSON("testData/cases/multiObject/mixedMesher.tessellator.json");
    EXPECT_EQ(objects.size(), 3);
    
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_FALSE(objects[0].isVolume);
    EXPECT_TRUE(objects[0].mesherOverride.has_value());
    EXPECT_EQ(objects[0].mesherOverride.value()["type"], "staircase");
    EXPECT_TRUE(objects[0].mesherOverride.value().contains("options"));
    EXPECT_TRUE(objects[0].mesherOverride.value()["options"].contains("compress"));
    EXPECT_TRUE(objects[0].mesherOverride.value()["options"]["compress"]);

    EXPECT_EQ(objects[1].filename, "cone.stl");
    EXPECT_FALSE(objects[1].isVolume);
    EXPECT_TRUE(objects[1].mesherOverride.has_value());
    EXPECT_EQ(objects[1].mesherOverride.value()["type"], "conformal");
    
    EXPECT_EQ(objects[2].filename, "cone.stl");
    EXPECT_FALSE(objects[2].isVolume);
    EXPECT_FALSE(objects[2].mesherOverride.has_value());
}

TEST_F(LauncherTest, readObjectsFromJSON_singleObject)
{
    auto objects = readObjectsFromJSON("testData/cases/multiObject/singleObject.tessellator.json");
    EXPECT_EQ(objects.size(), 1);
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_EQ(objects[0].group, "default_group");
    EXPECT_FALSE(objects[0].isVolume);
}

TEST_F(LauncherTest, readObjectsFromJSON_legacyFormat)
{
    auto objects = readObjectsFromJSON("testData/cases/sphere/closed_sphere.tessellator.json");
    EXPECT_EQ(objects.size(), 1);
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_EQ(objects[0].group, "sphere");
    EXPECT_TRUE(objects[0].isVolume);
}

TEST_F(LauncherTest, builds_staircased_mesher_with_override)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };

    std::string filename = "testData/cases/multiObject/mixedMesher.tessellator.json";
    
    auto objects = readObjectsFromJSON(filename);

    auto mesher = meshlib::app::buildMesher(meshMock, filename, objects[0]);
    
    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.isVolume, false);
    EXPECT_EQ(options.compress, true);
}

TEST_F(LauncherTest, launches_multiObject_basic)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/basic.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_mixedMesher)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/mixedMesher.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_singleObject)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/singleObject.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_sameFileMultipleGroups)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/sameFileMultipleGroups.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = meshlib::app::launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

