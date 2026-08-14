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
    EXPECT_EQ(launcher(ac, av), EXIT_SUCCESS);
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

    meshlib::Grid grid = parseGridFromJSON(j["grid"]);

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

TEST_F(LauncherTest, builds_staircased_mesher_default)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    std::string fileName = "testData/cases/longPolyline/longPolyline_legacy.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }

    ObjectDefinition objDef;
    auto mesher = buildMesher(meshMock, j, objDef);

    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.volumeGroups.size(), 0);
    EXPECT_EQ(options.compress, false);
    EXPECT_FALSE(options.splitHexahedra);
}

TEST_F(LauncherTest, buildsStaircasedMesherWithSplitHexahedra)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    nlohmann::json config = {
        {"mesher", {
            {"type", "staircase"},
            {"options", {{"splitHexahedra", true}}}
        }}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& staircase =
        dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher);

    EXPECT_TRUE(staircase.getOptions().splitHexahedra);
}

TEST_F(LauncherTest, builds_staircased_mesher_without_compression)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    std::string fileName = "testData/cases/longPolyline/longPolyline.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }

    ObjectDefinition objDef;
    auto mesher = buildMesher(meshMock, j, objDef);

    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.volumeGroups.size(), 0);
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
    std::string fileName = "testData/cases/longPolyline/longPolyline_compression.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }

    ObjectDefinition objDef;
    auto mesher = buildMesher(meshMock, j, objDef);
    
    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.volumeGroups.size(), 0);
    EXPECT_EQ(options.compress, true);
}

TEST_F(LauncherTest, launches_alhambra_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/alhambra/alhambra.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_alhambra_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/alhambra/alhambra.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}


TEST_F(LauncherTest, launches_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/sphere.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_closed_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/closed_sphere.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_sphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/sphere/sphere.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_thinCylinder_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/thinCylinder/thinCylinder.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_thinCylinder_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/thinCylinder/thinCylinder.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_cone_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/cone/cone.tessellator.json" };
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_long_polyline_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/longPolyline/longPolyline_legacy.tessellator.json" };
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_conformal_cone_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/cone/cone.conformal.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, readObjectsFromJSON_basic)
{
    std::string fileName = "testData/cases/multiObject/basic.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }
    auto objects = readObjectsFromJSON(j);
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
    std::string fileName = "testData/cases/multiObject/mixedMesher.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }
    auto objects = readObjectsFromJSON(j);
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
    std::string fileName = "testData/cases/multiObject/singleObject.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }
    auto objects = readObjectsFromJSON(j);
    EXPECT_EQ(objects.size(), 1);
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_EQ(objects[0].group, "default_group");
    EXPECT_FALSE(objects[0].isVolume);
}

TEST_F(LauncherTest, readObjectsFromJSON_legacyFormat)
{
    std::string fileName = "testData/cases/sphere/closed_sphere.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }
    auto objects = readObjectsFromJSON(j);
    EXPECT_EQ(objects.size(), 1);
    EXPECT_EQ(objects[0].filename, "sphere.stl");
    EXPECT_EQ(objects[0].group, "sphere");
    EXPECT_TRUE(objects[0].isVolume);
}

TEST_F(LauncherTest, readObjectsFromJSON_solenoid)
{
    std::ifstream input("testData/cases/solenoid/solenoid.tessellator.json");
    nlohmann::json config;
    input >> config;

    const auto objects = readObjectsFromJSON(config);

    ASSERT_EQ(objects.size(), 5);
    EXPECT_EQ(objects[0].filename, "solenoid.vtu");
    EXPECT_EQ(objects[0].group, "Solenoid");
    EXPECT_EQ(objects[1].filename, "BC.vtu");
    EXPECT_EQ(objects[1].group, "BC");
    EXPECT_EQ(objects[2].filename, "Generator.vtu");
    EXPECT_EQ(objects[2].group, "Generator");
    EXPECT_EQ(objects[3].filename, "Line.vtu");
    EXPECT_EQ(objects[3].group, "Line");
    EXPECT_EQ(objects[4].filename, "Line001.vtu");
    EXPECT_EQ(objects[4].group, "Line001");
}

TEST_F(LauncherTest, builds_staircased_mesher_with_override)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };

    std::string fileName = "testData/cases/multiObject/mixedMesher.tessellator.json";
    nlohmann::json j;
    {
        std::ifstream i(fileName);
        i >> j;
    }
    
    auto objects = readObjectsFromJSON(j);

    auto mesher = meshlib::app::buildMesher(meshMock, j, objects[0]);
    
    EXPECT_NO_THROW(auto staircaseMesher = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher));
    const auto & options = dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher).getOptions();
    EXPECT_EQ(options.volumeGroups.size(), 0);
    EXPECT_EQ(options.compress, true);
}

TEST_F(LauncherTest, launches_multiObject_basic)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/basic.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_mixedMesher)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/mixedMesher.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_singleObject)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/singleObject.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_multiObject_sameFileMultipleGroups)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/multiObject/sameFileMultipleGroups.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}

TEST_F(LauncherTest, launches_solenoid_multiObject_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/solenoid/solenoid.tessellator.json"};
    int exitCode;
    EXPECT_NO_THROW(exitCode = launcher(ac, av));
    EXPECT_EQ(exitCode, EXIT_SUCCESS);
}
