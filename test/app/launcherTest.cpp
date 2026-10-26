#include <gtest/gtest.h>

#include "app/launcher.h"
#include "MeshFixtures.h"
#include "types/Mesh.h"
#include "meshers/StaircaseMesher.h"
#include "meshers/ConformalMesher.h"
#include "core/VolumeShellExtractor.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"
#include "app/vtkIO.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <vector>

using namespace meshlib::app;

namespace {

class StderrCapture
{
public:
    StderrCapture() : previous_(std::cerr.rdbuf(buffer_.rdbuf())) {}
    ~StderrCapture() { std::cerr.rdbuf(previous_); }

    std::string str() const { return buffer_.str(); }

private:
    std::ostringstream buffer_;
    std::streambuf* previous_;
};

int callLauncher(const std::string& input, std::string& error)
{
    const char* arguments[] = {nullptr, "-i", input.c_str()};
    StderrCapture capture;
    const int exitCode = launcher(3, arguments);
    error = capture.str();
    return exitCode;
}

std::size_t countElements(const meshlib::Mesh& mesh, meshlib::Element::Type type)
{
    std::size_t count = 0;
    for (const meshlib::Group& group : mesh.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (element.type == type) {
                ++count;
            }
        }
    }
    return count;
}

bool pointInsideQuad(
    const meshlib::Coordinate& point,
    const meshlib::Element& quad,
    const meshlib::Coordinates& coordinates)
{
    if (!quad.isQuad()) {
        return false;
    }

    int planeAxis = -1;
    double planeValue = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        bool constant = true;
        const double value = coordinates[quad.vertices.front()][axis];
        for (const meshlib::CoordinateId vertex : quad.vertices) {
            if (std::abs(coordinates[vertex][axis] - value) > 1e-6) {
                constant = false;
                break;
            }
        }
        if (constant) {
            if (planeAxis != -1) {
                return false;
            }
            planeAxis = static_cast<int>(axis);
            planeValue = value;
        }
    }
    if (planeAxis < 0 || std::abs(point[planeAxis] - planeValue) > 1e-6) {
        return false;
    }

    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (static_cast<int>(axis) == planeAxis) {
            continue;
        }
        double minimum = std::numeric_limits<double>::max();
        double maximum = std::numeric_limits<double>::lowest();
        for (const meshlib::CoordinateId vertex : quad.vertices) {
            minimum = std::min(minimum, coordinates[vertex][axis]);
            maximum = std::max(maximum, coordinates[vertex][axis]);
        }
        if (point[axis] < minimum - 1e-6 || point[axis] > maximum + 1e-6) {
            return false;
        }
    }
    return true;
}

bool segmentInsideAnyQuad(
    const meshlib::Coordinate& first,
    const meshlib::Coordinate& second,
    const meshlib::Mesh& surface)
{
    for (const meshlib::Group& group : surface.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (!element.isQuad()) {
                continue;
            }
            if (pointInsideQuad(first, element, surface.coordinates)
                && pointInsideQuad(second, element, surface.coordinates)) {
                return true;
            }
        }
    }
    return false;
}

bool quadInsideAnyQuad(
    const meshlib::Element& quad,
    const meshlib::Mesh& quadMesh,
    const meshlib::Mesh& surface)
{
    for (const meshlib::Group& group : surface.groups) {
        for (const meshlib::Element& host : group.elements) {
            if (!host.isQuad()) {
                continue;
            }
            bool inside = true;
            for (const meshlib::CoordinateId vertex : quad.vertices) {
                if (!pointInsideQuad(
                        quadMesh.coordinates[vertex], host, surface.coordinates)) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                return true;
            }
        }
    }
    return false;
}

bool isConnectedStrip(const meshlib::Mesh& strip)
{
    std::vector<const meshlib::Element*> quads;
    for (const meshlib::Group& group : strip.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (element.isQuad()) {
                quads.push_back(&element);
            }
        }
    }
    if (quads.empty()) {
        return false;
    }

    std::map<std::pair<meshlib::Coordinate, meshlib::Coordinate>,
             std::vector<std::size_t>> edgeQuads;
    for (std::size_t quadId = 0; quadId < quads.size(); ++quadId) {
        const auto& vertices = quads[quadId]->vertices;
        for (std::size_t i = 0; i < 4; ++i) {
            meshlib::Coordinate first = strip.coordinates[vertices[i]];
            meshlib::Coordinate second = strip.coordinates[vertices[(i + 1) % 4]];
            if (second < first) {
                std::swap(first, second);
            }
            edgeQuads[{ first, second }].push_back(quadId);
        }
    }

    std::vector<std::set<std::size_t>> adjacency(quads.size());
    for (const auto& entry : edgeQuads) {
        const auto& ids = entry.second;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            for (std::size_t j = i + 1; j < ids.size(); ++j) {
                adjacency[ids[i]].insert(ids[j]);
                adjacency[ids[j]].insert(ids[i]);
            }
        }
    }

    std::set<std::size_t> visited;
    std::queue<std::size_t> pending;
    pending.push(0);
    while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop();
        if (!visited.insert(current).second) {
            continue;
        }
        for (const std::size_t neighbour : adjacency[current]) {
            pending.push(neighbour);
        }
    }
    return visited.size() == quads.size();
}

std::size_t maximumQuadsAtVertex(const meshlib::Mesh& strip)
{
    std::map<meshlib::Coordinate, std::size_t> uses;
    std::size_t maximum = 0;
    for (const meshlib::Group& group : strip.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (!element.isQuad()) {
                continue;
            }
            for (const meshlib::CoordinateId vertex : element.vertices) {
                maximum = std::max(
                    maximum, ++uses[strip.coordinates[vertex]]);
            }
        }
    }
    return maximum;
}

bool hasMixedSurfaceAndLowerDimensionalElements(const meshlib::Mesh& mesh)
{
    meshlib::utils::GridTools gridTools(mesh.grid);
    for (const meshlib::Group& group : mesh.groups) {
        const auto cellElements = gridTools.buildCellElemMap(
            group.elements, mesh.coordinates);
        for (const auto& [cell, elements] : cellElements) {
            const bool hasSurface = std::any_of(
                elements.begin(), elements.end(), [](const meshlib::Element* element) {
                    return element->isTriangle() || element->isQuad();
                });
            const bool hasLowerDimensionalElement = std::any_of(
                elements.begin(), elements.end(), [](const meshlib::Element* element) {
                    return element->isLine() || element->isNode();
                });
            if (hasSurface && hasLowerDimensionalElement) {
                return true;
            }
        }
    }
    return false;
}

void expectConformalVolumeLaunch(
    const std::string& input,
    const std::string& group)
{
    const char* arguments[] = {nullptr, "-i", input.c_str()};
    ASSERT_EQ(launcher(3, arguments), EXIT_SUCCESS);

    const auto output = std::filesystem::path(input).parent_path()
        / (group + ".tessellator.cmsh.vtk");
    const auto mesh = meshlib::vtkIO::readInputMesh(output);
    ASSERT_EQ(mesh.groups.size(), 1);
    EXPECT_GT(mesh.countElems(), 0);
}

} // namespace

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
    EXPECT_EQ(options.compress, true);
    EXPECT_FALSE(options.splitHexahedra);
    EXPECT_EQ(options.decimalPlacesInCollapser, 4);
    EXPECT_EQ(options.thinSlotMode, meshlib::meshers::ThinSlotMode::None);
    EXPECT_FALSE(options.thinSlotAllowOnSurfacePerimeter);
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

TEST_F(LauncherTest, buildsStaircasedMesherWithDecimalPlacesInCollapser)
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
            {"options", {{"decimalPlacesInCollapser", 2}}}
        }}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& staircase =
        dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher);

    EXPECT_EQ(staircase.getOptions().decimalPlacesInCollapser, 2);
}

TEST_F(LauncherTest, rejectsInvalidStaircaseDecimalPlacesInCollapser)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    ObjectDefinition object;

    for (const nlohmann::json& decimalPlaces : {
            nlohmann::json(true), nlohmann::json(1.0), nlohmann::json(-1)}) {
        const nlohmann::json config = {
            {"mesher", {{"type", "staircase"}, {"options", {{"decimalPlacesInCollapser", decimalPlaces}}}}}
        };
        EXPECT_THROW(buildMesher(meshMock, config, object), std::runtime_error);
    }
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

    EXPECT_TRUE(conformal.getOptions().compress);
    EXPECT_TRUE(conformal.getOptions().staircaseSharedCells);
    EXPECT_TRUE(conformal.getOptions().mergeAxisAlignedTriangles);
}

TEST_F(LauncherTest, conformalTriangleMergingCanBeDisabled)
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
            {"options", {
                {"compress", false},
                {"mergeAxisAlignedTriangles", false}
            }}
        }}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& conformal =
        dynamic_cast<meshlib::meshers::ConformalMesher&>(*mesher);

    EXPECT_FALSE(conformal.getOptions().compress);
    EXPECT_FALSE(conformal.getOptions().mergeAxisAlignedTriangles);
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

TEST_F(LauncherTest, solenoidSnapperCaseExportsNoMixedDimensionalCells)
{
    const auto sourceDirectory = std::filesystem::path(
        "testData/cases/solenoid");
    const auto outputDirectory = std::filesystem::temp_directory_path()
        / "tessellator_solenoid_snapper_launcher";
    std::filesystem::remove_all(outputDirectory);
    std::filesystem::copy(
        sourceDirectory,
        outputDirectory,
        std::filesystem::copy_options::recursive);

    const auto input = outputDirectory / "solenoid.conformal.tessellator.json";
    const std::string inputString = input.string();
    const char* argv[] = {nullptr, "-i", inputString.c_str()};
    EXPECT_EQ(launcher(3, argv), EXIT_SUCCESS);

    auto result = meshlib::vtkIO::readInputMesh(
        outputDirectory / "Solenoid.tessellator.cmsh.vtk");
    nlohmann::json inputData;
    std::ifstream(input) >> inputData;
    result.grid = parseGridFromJSON(inputData["grid"]);
    EXPECT_EQ(meshlib::utils::meshTools::countMeshElementsIf(
        result, meshlib::utils::meshTools::isNode), 0);
    EXPECT_FALSE(hasMixedSurfaceAndLowerDimensionalElements(result));
    EXPECT_FALSE(meshlib::meshFixtures::hasInvalidSurfaceAdjacency(result));

    std::filesystem::remove_all(outputDirectory);
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

TEST_F(LauncherTest, launchesAlhambraAsConformalVolume)
{
    expectConformalVolumeLaunch(
        "testData/cases/alhambra/alhambra.volume.conformal.tessellator.json",
        "alhambra");
}

TEST_F(LauncherTest, launchesSphereAsConformalVolume)
{
    expectConformalVolumeLaunch(
        "testData/cases/sphere/sphere.volume.conformal.tessellator.json",
        "sphere");
}

TEST_F(LauncherTest, launchesConeAsConformalVolume)
{
    expectConformalVolumeLaunch(
        "testData/cases/cone/cone.volume.conformal.tessellator.json",
        "cone");
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
    std::string error;
    EXPECT_EQ(callLauncher(inputString, error), EXIT_FAILURE);
    EXPECT_NE(error.find("duplicate group"), std::string::npos);

    std::filesystem::remove(input);
}

TEST_F(LauncherTest, missingInputFileReturnsError)
{
    const auto input = std::filesystem::temp_directory_path()
        / "tessellator_missing_input.tessellator.json";
    std::filesystem::remove(input);

    const std::string inputString = input.string();
    std::string error;
    EXPECT_EQ(callLauncher(inputString, error), EXIT_FAILURE);
    EXPECT_NE(error.find("Input file not found"), std::string::npos);
    EXPECT_NE(error.find(inputString), std::string::npos);
}

TEST_F(LauncherTest, malformedInputFileReturnsError)
{
    const auto input = std::filesystem::temp_directory_path()
        / "tessellator_malformed_input.tessellator.json";
    {
        std::ofstream stream(input);
        stream << "{ this is not valid JSON";
    }

    const std::string inputString = input.string();
    std::string error;
    EXPECT_EQ(callLauncher(inputString, error), EXIT_FAILURE);
    EXPECT_NE(error.find("Could not parse input file"), std::string::npos);
    EXPECT_NE(error.find(inputString), std::string::npos);

    std::filesystem::remove(input);
}

TEST_F(LauncherTest, missingObjectFileReturnsError)
{
    const auto directory = std::filesystem::temp_directory_path()
        / "tessellator_missing_object";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);

    const auto input = directory / "missing.tessellator.json";
    const nlohmann::json config = {
        {"grid", {
            {"numberOfCells", {2, 2, 2}},
            {"boundingBox", {{0, 0, 0}, {1, 1, 1}}}
        }},
        {"objects", {
            {{"filename", "does_not_exist.stl"}, {"group", "missing"}}
        }}
    };
    {
        std::ofstream stream(input);
        stream << config;
    }

    std::string error;
    EXPECT_EQ(callLauncher(input.string(), error), EXIT_FAILURE);
    EXPECT_NE(error.find("Object file not found"), std::string::npos);
    EXPECT_NE(error.find("does_not_exist.stl"), std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(directory / "missing.tessellator.str.vtk"));
    EXPECT_FALSE(std::filesystem::exists(directory / "missing.tessellator.grid.vtk"));

    std::filesystem::remove_all(directory);
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

TEST_F(LauncherTest, parsesThinSlotModeAndPerimeterOption)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    const nlohmann::json config = {
        {"mesher", {
            {"type", "staircase"},
            {"options", {
                {"thinSlot", "surface"},
                {"thinSlotAllowOnSurfacePerimeter", true}
            }}
        }}
    };

    ObjectDefinition object;
    auto mesher = buildMesher(meshMock, config, object);
    const auto& staircase =
        dynamic_cast<meshlib::meshers::StaircaseMesher&>(*mesher);
    EXPECT_EQ(
        staircase.getOptions().thinSlotMode,
        meshlib::meshers::ThinSlotMode::Surface);
    EXPECT_TRUE(staircase.getOptions().thinSlotAllowOnSurfacePerimeter);
}

TEST_F(LauncherTest, rejectsInvalidThinSlotMode)
{
    meshlib::Mesh meshMock;
    meshMock.grid = {
        std::vector<double>{0, 1},
        std::vector<double>{0, 1},
        std::vector<double>{0, 1}
    };
    const nlohmann::json config = {
        {"mesher", {
            {"type", "staircase"},
            {"options", {{"thinSlot", "quads"}}}
        }}
    };

    ObjectDefinition object;
    EXPECT_THROW(buildMesher(meshMock, config, object), std::runtime_error);
}

TEST_F(LauncherTest, thinSlotLineOnSquareInSquareIsEmbedded)
{
    const std::string input =
        "testData/cases/thinSlot/squareInSquare/"
        "squareInSquare.thinSlotLine.tessellator.json";
    std::string error;
    ASSERT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;

    const std::filesystem::path folder =
        std::filesystem::path(input).parent_path();
    const auto host = meshlib::vtkIO::readInputMesh(
        folder / "SquareInSquare.tessellator.str.vtk");
    const auto slot = meshlib::vtkIO::readInputMesh(
        folder / "Discretize_Wire.tessellator.str.vtk");

    EXPECT_EQ(countElements(slot, meshlib::Element::Type::Surface), 0u);
    EXPECT_GT(countElements(slot, meshlib::Element::Type::Line), 0u);

    for (const meshlib::Group& group : slot.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (!element.isLine()) {
                continue;
            }
            const auto& first = slot.coordinates[element.vertices[0]];
            const auto& second = slot.coordinates[element.vertices[1]];
            EXPECT_TRUE(segmentInsideAnyQuad(first, second, host))
                << first.str() << " -> " << second.str();
        }
    }
}

TEST_F(LauncherTest, thinSlotLineOnFineGridIsEmbedded)
{
    const std::string input =
        "testData/cases/thinSlot/squareInSquare/"
        "squareInSquare.thinSlotLine.fine.tessellator.json";
    std::string error;
    ASSERT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;

    const std::filesystem::path folder =
        std::filesystem::path(input).parent_path();
    const auto host = meshlib::vtkIO::readInputMesh(
        folder / "SquareInSquare.tessellator.str.vtk");
    const auto slot = meshlib::vtkIO::readInputMesh(
        folder / "Discretize_Wire.tessellator.str.vtk");

    for (const meshlib::Group& group : slot.groups) {
        for (const meshlib::Element& element : group.elements) {
            if (!element.isLine()) {
                continue;
            }
            const auto& first = slot.coordinates[element.vertices[0]];
            const auto& second = slot.coordinates[element.vertices[1]];
            EXPECT_TRUE(segmentInsideAnyQuad(first, second, host))
                << first.str() << " -> " << second.str();
        }
    }
}

TEST_F(LauncherTest, thinSlotSurfaceOnSquareInSquareIsContinuousStrip)
{
    const std::string input =
        "testData/cases/thinSlot/squareInSquare/"
        "squareInSquare.thinSlotSurface.tessellator.json";
    std::string error;
    ASSERT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;

    const std::filesystem::path folder =
        std::filesystem::path(input).parent_path();
    const auto host = meshlib::vtkIO::readInputMesh(
        folder / "SquareInSquare.tessellator.str.vtk");
    const auto slot = meshlib::vtkIO::readInputMesh(
        folder / "Discretize_Wire.tessellator.str.vtk");

    EXPECT_EQ(
        countElements(slot, meshlib::Element::Type::Line), 0u);
    EXPECT_EQ(
        countElements(slot, meshlib::Element::Type::Surface),
        slot.countElems());
    EXPECT_GT(countElements(slot, meshlib::Element::Type::Surface), 0u);

    for (const meshlib::Group& group : slot.groups) {
        for (const meshlib::Element& element : group.elements) {
            EXPECT_TRUE(element.isQuad());
            EXPECT_TRUE(quadInsideAnyQuad(element, slot, host));
        }
    }
    EXPECT_TRUE(isConnectedStrip(slot));
    EXPECT_LE(maximumQuadsAtVertex(slot), 3u);
}

TEST_F(LauncherTest, thinSlotLineRejectsSurfacePerimeterByDefault)
{
    const std::string input =
        "testData/cases/thinSlot/perimeter/"
        "perimeter.thinSlotLine.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_FAILURE);
    EXPECT_NE(error.find("perimeter"), std::string::npos);
}

TEST_F(LauncherTest, thinSlotLineAllowsSurfacePerimeterWhenConfigured)
{
    const std::string input =
        "testData/cases/thinSlot/perimeter/"
        "perimeter.thinSlotLine.allowed.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;
}

TEST_F(LauncherTest, thinSlotSurfaceRejectsSurfacePerimeterByDefault)
{
    const std::string input =
        "testData/cases/thinSlot/perimeter/"
        "perimeter.thinSlotSurface.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_FAILURE);
    EXPECT_NE(error.find("perimeter"), std::string::npos);
}

TEST_F(LauncherTest, thinSlotLineRejectsUnembeddableGrid)
{
    const std::string input =
        "testData/cases/thinSlot/squareInSquare/"
        "squareInSquare.thinSlotLine.incompatible.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_FAILURE);
    EXPECT_NE(error.find("not embedded"), std::string::npos);
}

TEST_F(LauncherTest, thinSlotLineOnInteriorEdgeIsAllowedByDefault)
{
    const std::string input =
        "testData/cases/thinSlot/perimeter/"
        "interior.thinSlotLine.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;
}

TEST_F(LauncherTest, globalThinSlotOptionSkipsHostSurfaceObjects)
{
    const std::string input =
        "testData/cases/thinSlot/squareInSquare/"
        "squareInSquare.thinSlotLine.global.tessellator.json";
    std::string error;
    EXPECT_EQ(callLauncher(input, error), EXIT_SUCCESS) << error;
}
