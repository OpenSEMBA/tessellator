#include <gtest/gtest.h>

#include "app/launcher.h"
#include "types/Mesh.h"
#include "meshers/StaircaseMesher.h"
#include "meshers/ConformalMesher.h"
#include "core/VolumeShellExtractor.h"
#include "utils/MeshTools.h"
#include "app/vtkIO.h"

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

TEST_F(LauncherTest, singleFileOutputIsDisabledByDefault)
{
    EXPECT_FALSE(readSingleFileOutputOption(nlohmann::json::object()));
    EXPECT_TRUE(readSingleFileOutputOption({
        {"output", {{"singleFile", true}}}
    }));
}

TEST_F(LauncherTest, conformalMesherStaircasesSharedCellsByDefault)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    const nlohmann::json config = {
        {"mesher", {{"type", "conformal"}}}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& conformal =
        dynamic_cast<meshlib::meshers::ConformalMesher&>(*mesher);

    EXPECT_TRUE(conformal.getOptions().staircaseSharedCells);
}

TEST_F(LauncherTest, conformalSharedCellStaircasingCanBeDisabled)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    const nlohmann::json config = {
        {"mesher", {
            {"type", "conformal"},
            {"options", {{"staircaseSharedCells", false}}}
        }}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& conformal =
        dynamic_cast<meshlib::meshers::ConformalMesher&>(*mesher);

    EXPECT_FALSE(conformal.getOptions().staircaseSharedCells);
}

TEST_F(LauncherTest, parsesConformalSnapOptions)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    const nlohmann::json config = {
        {"mesher", {
            {"type", "conformal"},
            {"options", {{"edgePoints", 3}, {"forbiddenLength", 0.25}}}
        }}
    };

    ObjectDefinition object;
    const auto mesher = buildMesher(meshMock, config, object);
    const auto& conformal =
        dynamic_cast<meshlib::meshers::ConformalMesher&>(*mesher);

    EXPECT_EQ(conformal.getOptions().snapperOptions.edgePoints, 3);
    EXPECT_DOUBLE_EQ(conformal.getOptions().snapperOptions.forbiddenLength, 0.25);
}

TEST_F(LauncherTest, rejectsInvalidConformalSnapOptions)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    ObjectDefinition object;

    for (const nlohmann::json& edgePoints : {
            nlohmann::json(true), nlohmann::json(1.0), nlohmann::json(-1)}) {
        const nlohmann::json config = {
            {"mesher", {{"type", "conformal"}, {"options", {{"edgePoints", edgePoints}}}}}
        };
        EXPECT_THROW(buildMesher(meshMock, config, object), std::runtime_error);
    }

    for (const nlohmann::json& forbiddenLength : {
            nlohmann::json(-0.1), nlohmann::json(0.6), nlohmann::json("invalid")}) {
        const nlohmann::json config = {
            {"mesher", {{"type", "conformal"}, {"options", {{"forbiddenLength", forbiddenLength}}}}}
        };
        EXPECT_THROW(buildMesher(meshMock, config, object), std::runtime_error);
    }
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

TEST_F(LauncherTest, launches_conformal_smallSphere_case)
{
    int ac = 3;
    const char* av[] = { NULL, "-i", "testData/cases/smallSphere/smallSphere.conformal.tessellator.json"};
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

TEST_F(LauncherTest, launchesTypicalCasesAsStaircaseVolumes)
{
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"testData/cases/alhambra/alhambra.volume.tessellator.json", "alhambra"},
        {"testData/cases/sphere/sphere.volume.tessellator.json", "sphere"},
        {"testData/cases/cone/cone.volume.tessellator.json", "cone"}
    };

    for (const auto& [input, group] : cases) {
        SCOPED_TRACE(input);
        const char* arguments[] = {nullptr, "-i", input.c_str()};
        ASSERT_EQ(launcher(3, arguments), EXIT_SUCCESS);

        const auto output = std::filesystem::path(input).parent_path()
            / (group + ".tessellator.str.vtk");
        const auto mesh = meshlib::vtkIO::readInputMesh(output);
        ASSERT_GT(mesh.countElems(), 0);
        EXPECT_EQ(mesh.countElems(), meshlib::utils::meshTools::countMeshElementsIf(
            mesh, meshlib::utils::meshTools::isHexahedron));
    }
}

TEST_F(LauncherTest, launchesTypicalCasesAsConformalVolumes)
{
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"testData/cases/alhambra/alhambra.volume.conformal.tessellator.json", "alhambra"},
        {"testData/cases/sphere/sphere.volume.conformal.tessellator.json", "sphere"},
        {"testData/cases/cone/cone.volume.conformal.tessellator.json", "cone"}
    };

    for (const auto& [input, group] : cases) {
        SCOPED_TRACE(input);
        const char* arguments[] = {nullptr, "-i", input.c_str()};
        ASSERT_EQ(launcher(3, arguments), EXIT_SUCCESS);

        const auto output = std::filesystem::path(input).parent_path()
            / (group + ".tessellator.cmsh.vtk");
        const auto mesh = meshlib::vtkIO::readInputMesh(output);
        ASSERT_EQ(mesh.groups.size(), 1);
        EXPECT_GT(mesh.countElems(), 0);
    }
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
    EXPECT_FALSE(objects[0].ghost);
    EXPECT_FALSE(objects[0].mesherOverride.has_value());
    EXPECT_EQ(objects[1].filename, "cone.stl");
    EXPECT_EQ(objects[1].group, "cone_group");
    EXPECT_FALSE(objects[1].isVolume);
    EXPECT_FALSE(objects[1].ghost);
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
    EXPECT_FALSE(objects[0].ghost);
    ASSERT_TRUE(objects[0].mesherOverride.has_value());
    EXPECT_EQ(objects[0].mesherOverride.value()["type"], "conformal");
    EXPECT_EQ(objects[1].filename, "BC.vtu");
    EXPECT_EQ(objects[1].group, "BC");
    EXPECT_TRUE(objects[1].ghost);
    EXPECT_EQ(objects[2].filename, "Generator.vtu");
    EXPECT_EQ(objects[2].group, "Generator");
    EXPECT_FALSE(objects[2].ghost);
    EXPECT_EQ(objects[3].filename, "wire_left.vtu");
    EXPECT_EQ(objects[3].group, "Wire_left");
    EXPECT_FALSE(objects[3].ghost);
    EXPECT_EQ(objects[4].filename, "wire_right.vtu");
    EXPECT_EQ(objects[4].group, "Wire_right");
    EXPECT_FALSE(objects[4].ghost);

    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    auto solenoidMesher = buildMesher(meshMock, config, objects[0]);
    EXPECT_NE(
        dynamic_cast<meshlib::meshers::ConformalMesher*>(solenoidMesher.get()),
        nullptr);
    for (std::size_t objectIndex = 1; objectIndex < objects.size(); ++objectIndex) {
        auto mesher = buildMesher(meshMock, config, objects[objectIndex]);
        EXPECT_NE(
            dynamic_cast<meshlib::meshers::StaircaseMesher*>(mesher.get()),
            nullptr);
    }
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

TEST_F(LauncherTest, launchesMultiObjectIntoSingleGroupedFile)
{
    const auto temp = std::filesystem::temp_directory_path();
    const auto input = temp / "tessellator_single_file.tessellator.json";
    const auto output = temp / "tessellator_single_file.tessellator.vtk";
    const auto gridOutput = temp / "tessellator_single_file.tessellator.grid.vtk";
    const auto sphereOutput = temp / "sphere_group.tessellator.str.vtk";
    const auto coneOutput = temp / "cone_group.tessellator.str.vtk";
    std::filesystem::remove(output);
    std::filesystem::remove(gridOutput);
    std::filesystem::remove(sphereOutput);
    std::filesystem::remove(coneOutput);
    const nlohmann::json config = {
        {"grid", {
            {"numberOfCells", {10, 10, 10}},
            {"boundingBox", {{-100, -100, -100}, {100, 100, 100}}}
        }},
        {"mesher", {{"type", "staircase"}}},
        {"output", {{"singleFile", true}}},
        {"objects", {
            {
                {"filename", std::filesystem::absolute(
                    "testData/cases/multiObject/sphere.stl").string()},
                {"volume", true},
                {"group", "sphere_group"}
            },
            {
                {"filename", std::filesystem::absolute(
                    "testData/cases/multiObject/cone.stl").string()},
                {"group", "cone_group"}
            }
        }}
    };
    {
        std::ofstream stream(input);
        stream << config;
    }

    const std::string inputString = input.string();
    const char* av[] = {nullptr, "-i", inputString.c_str()};
    EXPECT_EQ(launcher(3, av), EXIT_SUCCESS);

    ASSERT_TRUE(std::filesystem::exists(output));
    EXPECT_FALSE(std::filesystem::exists(sphereOutput));
    EXPECT_FALSE(std::filesystem::exists(coneOutput));
    {
        std::ifstream stream(output);
        const std::string contents{
            std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
        EXPECT_NE(contents.find("groupNames"), std::string::npos);
        EXPECT_NE(contents.find("sphere_group"), std::string::npos);
        EXPECT_NE(contents.find("cone_group"), std::string::npos);
    }

    std::filesystem::remove(output);
    std::filesystem::remove(gridOutput);
    std::filesystem::remove(input);
}

TEST_F(LauncherTest, rejectsDuplicateGroupNamesInSingleFileOutput)
{
    const auto input = std::filesystem::temp_directory_path()
        / "tessellator_duplicate_groups.json";
    const nlohmann::json config = {
        {"grid", {
            {"numberOfCells", {1, 1, 1}},
            {"boundingBox", {{0, 0, 0}, {1, 1, 1}}}
        }},
        {"output", {{"singleFile", true}}},
        {"objects", {
            {{"filename", "first.stl"}, {"group", "duplicate"}},
            {{"filename", "second.stl"}, {"group", "duplicate"}}
        }}
    };
    {
        std::ofstream stream(input);
        stream << config;
    }
    const std::string inputString = input.string();
    const char* av[] = {nullptr, "-i", inputString.c_str()};

    EXPECT_THROW(launcher(3, av), std::runtime_error);

    std::filesystem::remove(input);
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
