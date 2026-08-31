#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include <array>
#include <map>
#include <queue>
#include <set>

#include "meshers/StaircaseMesher.h"
#include "StaircaseMesherOptions.h"
#include "Staircaser.h"

#include "core/Slicer.h"
#include "core/Collapser.h"

#include "utils/GridTools.h"
#include "utils/MeshTools.h"
#include "utils/RedundancyCleaner.h"


#if APP_LOADED
    #include "app/vtkIO.h"
#endif

namespace meshlib::meshers {

using namespace meshFixtures;
using namespace utils;
using namespace meshTools;

namespace {

using EdgeKey = std::array<CoordinateId, 2>;
using FaceKey = std::array<CoordinateId, 4>;

EdgeKey edgeKey(CoordinateId first, CoordinateId second)
{
    return first < second ? EdgeKey{first, second} : EdgeKey{second, first};
}

bool isSingleClosedSurface(const Elements& faces)
{
    if (faces.empty()) {
        return false;
    }
    std::map<EdgeKey, std::vector<ElementId>> edgeFaces;
    for (ElementId faceId = 0; faceId < faces.size(); ++faceId) {
        const auto& vertices = faces[faceId].vertices;
        if (vertices.size() < 3) {
            return false;
        }
        for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
            edgeFaces[edgeKey(vertices[vertex], vertices[(vertex + 1) % vertices.size()])]
                .push_back(faceId);
        }
    }

    std::vector<std::set<ElementId>> adjacency(faces.size());
    for (const auto& entry : edgeFaces) {
        if (entry.second.size() != 2) {
            return false;
        }
        const ElementId first = entry.second[0];
        const ElementId second = entry.second[1];
        adjacency[first].insert(second);
        adjacency[second].insert(first);
    }

    std::set<ElementId> visited{0};
    std::queue<ElementId> pending;
    pending.push(0);
    while (!pending.empty()) {
        const ElementId current = pending.front();
        pending.pop();
        for (ElementId neighbor : adjacency[current]) {
            if (visited.insert(neighbor).second) {
                pending.push(neighbor);
            }
        }
    }
    return visited.size() == faces.size();
}

bool isSingleClosedHexahedralVolume(const Mesh& mesh)
{
    static const std::array<std::array<std::size_t, 4>, 6> hexahedronFaces{{
        {{0, 1, 2, 3}}, {{4, 5, 6, 7}},
        {{0, 1, 5, 4}}, {{1, 2, 6, 5}},
        {{2, 3, 7, 6}}, {{3, 0, 4, 7}}
    }};

    std::vector<const Element*> hexahedra;
    for (const auto& group : mesh.groups) {
        for (const auto& element : group.elements) {
            if (!element.isHexahedron()) {
                return false;
            }
            hexahedra.push_back(&element);
        }
    }
    if (hexahedra.empty()) {
        return false;
    }

    struct FaceOccurrence {
        ElementId hexahedron;
        CoordinateIds vertices;
    };
    std::map<FaceKey, std::vector<FaceOccurrence>> faceOccurrences;
    for (ElementId elementId = 0; elementId < hexahedra.size(); ++elementId) {
        for (const auto& face : hexahedronFaces) {
            CoordinateIds vertices;
            FaceKey key;
            for (std::size_t vertex = 0; vertex < face.size(); ++vertex) {
                vertices.push_back(hexahedra[elementId]->vertices[face[vertex]]);
                key[vertex] = vertices.back();
            }
            std::sort(key.begin(), key.end());
            faceOccurrences[key].push_back({elementId, vertices});
        }
    }

    Elements boundary;
    std::vector<std::set<ElementId>> adjacency(hexahedra.size());
    for (const auto& entry : faceOccurrences) {
        if (entry.second.size() == 1) {
            boundary.emplace_back(
                entry.second.front().vertices, Element::Type::Surface);
        } else if (entry.second.size() == 2) {
            const ElementId first = entry.second[0].hexahedron;
            const ElementId second = entry.second[1].hexahedron;
            adjacency[first].insert(second);
            adjacency[second].insert(first);
        } else {
            return false;
        }
    }

    std::set<ElementId> visited{0};
    std::queue<ElementId> pending;
    pending.push(0);
    while (!pending.empty()) {
        const ElementId current = pending.front();
        pending.pop();
        for (ElementId neighbor : adjacency[current]) {
            if (visited.insert(neighbor).second) {
                pending.push(neighbor);
            }
        }
    }
    return visited.size() == hexahedra.size()
        && isSingleClosedSurface(boundary);
}

}


class StaircaseMesherTest : public ::testing::Test {
public:
    static std::size_t countRepeatedElements(const Mesh& mesh)
    {
        std::set<std::set<CoordinateId>> verticesSets;
        std::set<CoordinateIds> lineVerticesSets;
        for (auto const& g : mesh.groups) {
            for (auto const& e : g.elements) {
                if (e.isLine()) {
                    lineVerticesSets.insert(e.vertices);
                }
                else {
                    verticesSets.insert(std::set<CoordinateId>(e.vertices.begin(), e.vertices.end()));  
                }
            }
        }
        return mesh.countElems() - verticesSets.size() - lineVerticesSets.size();
    }

    static void assertMeshEqual(const Mesh& leftMesh, const Mesh& rightMesh) {
        for (Axis axis : { X, Y, Z }) {
            auto& leftGridAxisPlanes = leftMesh.grid[axis];
            auto& rightGridAxisPlanes = rightMesh.grid[axis];

            EXPECT_EQ(leftGridAxisPlanes.size(), rightGridAxisPlanes.size()) << "Current Axis: #" << axis << std::endl;

            for (std::size_t plane = 0; plane != leftGridAxisPlanes.size(); ++plane) {
                EXPECT_EQ(leftGridAxisPlanes[plane], rightGridAxisPlanes[plane])
                    << "Current Axis: #" << axis << std::endl
                    << "Current Plane: #" << plane << std::endl;
            }
        }

        ASSERT_EQ(leftMesh.coordinates.size(), rightMesh.coordinates.size());

        for (CoordinateId id = 0; id < leftMesh.coordinates.size(); ++id) {
            for (auto axis : { X, Y, Z }) {
                EXPECT_EQ(leftMesh.coordinates[id][axis], rightMesh.coordinates[id][axis]);
            }
        }

        ASSERT_EQ(leftMesh.groups.size(), rightMesh.groups.size());

        for (std::size_t g = 0; g < leftMesh.groups.size(); ++g) {
            auto& leftGroup = leftMesh.groups[g];
            auto& rightGroup = rightMesh.groups[g];

            ASSERT_EQ(leftGroup.elements.size(), rightGroup.elements.size()) << "Current Group: #" << g << std::endl;

            for (std::size_t e = 0; e < leftGroup.elements.size(); ++e) {
                auto& resultElement = leftGroup.elements[e];
                auto& expectedElement = rightGroup.elements[e];
                EXPECT_EQ(resultElement.type, expectedElement.type)
                    << "Current Group: #" << g << std::endl
                    << "Current Element: #" << e << std::endl;

                for (CoordinateId v = 0; v != resultElement.vertices.size(); ++v) {
                    EXPECT_EQ(resultElement.vertices[v], expectedElement.vertices[v])
                        << "Current Group: #" << g << std::endl
                        << "Current Element: #" << e << std::endl
                        << "Current Vertex: #" << v << std::endl;
                }

            }
        }
    }
};

TEST_F(StaircaseMesherTest, testStaircaseLinesWithUniformGrid)
{

    const int numberOfCells = 4;
    const float step = 0.25;
    const float offset = 0.5;
    const float lowerCoordinateValue = -0.5;
    const float upperCoordinateValue =  0.5;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
    inputMesh.coordinates = {
        Coordinate({-0.475, -0.267, 0.452647}),
        Coordinate({ 0.4, 0.1, -0.2}),
    };
    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 0}, Element::Type::Line)
    };

    Mesh expectedMesh;
    expectedMesh.grid = inputMesh.grid;
    expectedMesh.coordinates = {
        Coordinate({0.00, 1.00, 4.00}),     // 0
        Coordinate({1.00, 1.00, 4.00}),     // 1
        Coordinate({1.00, 1.00, 3.00}),     // 2
        Coordinate({1.00, 2.00, 3.00}),     // 3
        Coordinate({2.00, 2.00, 3.00}),     // 4
        Coordinate({2.00, 2.00, 2.00}),     // 5
        Coordinate({3.00, 2.00, 2.00}),     // 6
        Coordinate({3.00, 2.00, 1.00}),     // 7
        Coordinate({4.00, 2.00, 1.00}),     // 8
    };
    for (auto& c: expectedMesh.coordinates) {
        c *= step;
        c -= offset;
    }
    expectedMesh.groups.resize(1);
    expectedMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2, 3}, Element::Type::Line),
        Element({3, 4}, Element::Type::Line),
        Element({4, 5}, Element::Type::Line),
        Element({5, 6}, Element::Type::Line),
        Element({6, 7}, Element::Type::Line),
        Element({7, 8}, Element::Type::Line),
        Element({8, 7}, Element::Type::Line),
        Element({7, 6}, Element::Type::Line),
        Element({6, 5}, Element::Type::Line),
        Element({5, 4}, Element::Type::Line),
        Element({4, 3}, Element::Type::Line),
        Element({3, 2}, Element::Type::Line),
        Element({2, 1}, Element::Type::Line),
        Element({1, 0}, Element::Type::Line),
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = StaircaseMesher(inputMesh, 2).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));

    assertMeshEqual(resultMesh, expectedMesh);
}


TEST_F(StaircaseMesherTest, testStaircaseLinesWithRectilinearGrid)
{
    Mesh inputMesh;
    inputMesh.grid = Grid(
        {
            std::vector<CoordinateDir>({0.0, 1.0, 2.0}),
            std::vector<CoordinateDir>({-2.0, 0.0, 0.5}),
            std::vector<CoordinateDir>({0.0, 2.0, 6.0})
        }
    );
    inputMesh.coordinates = {
        Coordinate({0.13, -1.3, 0.1}),
        Coordinate({0.9, -0.5, 2.3}),
        Coordinate({1.4, 0.3, 5.5}),
    };

    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2, 1}, Element::Type::Line),
        Element({1, 0}, Element::Type::Line),
    };


    Mesh expectedMesh;
    expectedMesh.grid = inputMesh.grid;
    expectedMesh.coordinates = {
        Coordinate({0.00, 0.00, 0.00}),     // 0
        Coordinate({0.00, 1.00, 0.00}),     // 1
        Coordinate({0.00, 1.00, 1.00}),     // 2
        Coordinate({1.00, 1.00, 1.00}),     // 3
        Coordinate({1.00, 1.00, 2.00}),     // 4
        Coordinate({1.00, 2.00, 2.00}),     // 5
    };
    expectedMesh.coordinates = utils::GridTools{
        expectedMesh.grid}.relativeToAbsolute(expectedMesh.coordinates);
    expectedMesh.groups.resize(1);
    expectedMesh.groups[0].elements = {
        Element({0, 1}, Element::Type::Line),
        Element({1, 2}, Element::Type::Line),
        Element({2, 3}, Element::Type::Line),
        Element({3, 4}, Element::Type::Line),
        Element({4, 5}, Element::Type::Line),
        Element({5, 4}, Element::Type::Line),
        Element({4, 3}, Element::Type::Line),
        Element({3, 2}, Element::Type::Line),
        Element({2, 1}, Element::Type::Line),
        Element({1, 0}, Element::Type::Line),
    };

    Mesh resultMesh;
    ASSERT_NO_THROW(resultMesh = StaircaseMesher(inputMesh, 2).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));

    assertMeshEqual(resultMesh, expectedMesh);
}

TEST_F(StaircaseMesherTest, testTriNonUniformGridStaircase)
{
    Mesh out;
    StaircaseMesherOptions options;
    options.compress = false;
    ASSERT_NO_THROW(out = StaircaseMesher(buildTriNonUniformGridMesh(), 4, options).mesh());

    EXPECT_EQ(0, countRepeatedElements(out));
    EXPECT_EQ(6, out.groups[0].elements.size());
    EXPECT_EQ(4, countMeshElementsIf(out, isQuad));
    EXPECT_EQ(2, countMeshElementsIf(out, isLine));
    EXPECT_EQ(0, countMeshElementsIf(out, isNode));
}

#if APP_LOADED
// FOR DEBUG ONLY / OBTAIN VISUAL REPRESENTATION

TEST_F(StaircaseMesherTest, DISABLED_visualSelectiveStaircaserCone)
{
    // Input
    const std::string inputFilename = "testData/cases/cone/cone.stl";
    auto inputMesh = vtkIO::readInputMesh(inputFilename);

    inputMesh.grid[X] = utils::GridTools::linspace(-2.0,  2.0,  41); 
    inputMesh.grid[Y] = utils::GridTools::linspace(-2.0,  2.0,  41); 
    inputMesh.grid[Z] = utils::GridTools::linspace(-1.0, 11.0, 121);

    // SurfaceMesh

    auto surfaceMesh = meshlib::utils::meshTools::buildMeshFilteringElements(inputMesh, meshlib::utils::meshTools::isNotTetrahedron);

    // Slicer

    auto slicedMesh = meshlib::core::Slicer{surfaceMesh}.getMesh();

    // Collapser

    auto collapsedMesh = meshlib::core::Collapser{slicedMesh, 4}.getMesh();

    // Selection the specific cells to staircase and generate the result Mesh

    std::set<Cell> cellSet;

    for (int x = 0; x < 41; ++x) {
        for (int y = 0; y < 41; ++y) {
            for (int z = 0; z < 61; ++z) {  
                cellSet.insert(Cell{x, y, z});
            }
        }
    }

    auto resultMesh = meshlib::core::Staircaser{ collapsedMesh }.getSelectiveMesh(cellSet);
    ASSERT_NO_THROW(meshTools::checkNoCellsAreCrossed(resultMesh));

    RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(resultMesh);
    utils::meshTools::reduceGrid(resultMesh, inputMesh.grid);
    utils::meshTools::convertToAbsoluteCoordinates(resultMesh);

    EXPECT_TRUE(meshTools::isAClosedTopology(inputMesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(surfaceMesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(slicedMesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(resultMesh.groups[0].elements));



    std::filesystem::path outputFolder = meshlib::vtkIO::getFolder(inputFilename);
    auto basename = meshlib::vtkIO::getBasename(inputFilename);
    meshlib::vtkIO::exportMeshToVTU(outputFolder / (basename + ".tessellator.selective.vtk"), resultMesh);
    meshlib::vtkIO::exportGridToVTU(outputFolder / (basename + ".tessellator.selective.grid.vtk"), resultMesh.grid);
}

#endif

TEST_F(StaircaseMesherTest, testStaircaseTriangleWithUniformGrid)
{

    float lowerCoordinateValue = -0.5;
    float upperCoordinateValue = 0.5;
    int numberOfCells = 4;
    float step = 0.25;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
    inputMesh.coordinates = {
        Coordinate({-0.475, -0.15, -0.480}),
        Coordinate({ 0.475, -0.15, -0.480}),
        Coordinate({ 0.0, 0.15, 0.30}),
    };
    inputMesh.groups.resize(1);
    inputMesh.groups[0].elements = {  
        Element({0, 1, 2}, Element::Type::Surface)
    };

    Mesh resultMesh;
    StaircaseMesherOptions options;
    options.compress = false;
    ASSERT_NO_THROW(resultMesh = StaircaseMesher(inputMesh, 2, options).mesh());

    EXPECT_EQ(0, countRepeatedElements(resultMesh));
    EXPECT_EQ(12, resultMesh.groups[0].elements.size());
    EXPECT_EQ(10, countMeshElementsIf(resultMesh, isQuad));
    EXPECT_EQ(2, countMeshElementsIf(resultMesh, isLine)); 
    EXPECT_EQ(0, countMeshElementsIf(resultMesh, isNode));
}

TEST_F(StaircaseMesherTest, testStaircaseWithCompression)
{
    float lowerCoordinateValue = -0.5;
    float upperCoordinateValue = 0.5;
    int numberOfCells = 4;
    float step = 0.25;
    assert((upperCoordinateValue - lowerCoordinateValue) / (numberOfCells) == step);

    Mesh inputMesh;
    inputMesh.grid = GridTools::buildCartesianGrid(lowerCoordinateValue, upperCoordinateValue, numberOfCells + 1);
    inputMesh.coordinates = {
        Coordinate({ -0.475, -0.15, 0     }),
        Coordinate({  0.475, -0.15, 0     }),
        Coordinate({  0.0  ,  0.15, 0     }),
        Coordinate({  0.0  ,  0.15, 0.475 })
    };
    inputMesh.groups.resize(2);
    inputMesh.groups[0].elements = {  
        Element({0, 1, 2}, Element::Type::Surface),
        Element({2, 3}, Element::Type::Line)
    };
    inputMesh.groups[1].elements = {  
        Element({0, 2}, Element::Type::Line),
        Element({2, 3}, Element::Type::Line)
    };

    StaircaseMesherOptions nonCompressionOption;
    nonCompressionOption.compress = false;
    Mesh nonCompressedMesh = StaircaseMesher(inputMesh, 2, nonCompressionOption).mesh();
    Mesh compressedMesh;
    StaircaseMesherOptions compressOption;
    compressOption.compress = true;
    ASSERT_NO_THROW(compressedMesh = StaircaseMesher(inputMesh, 2, compressOption).mesh());

    EXPECT_EQ(3, countRepeatedElements(nonCompressedMesh));
    EXPECT_EQ(7, nonCompressedMesh.groups[0].elements.size());
    EXPECT_EQ(6, nonCompressedMesh.groups[1].elements.size());
    EXPECT_EQ(4, countMeshElementsIf(nonCompressedMesh, isQuad));
    EXPECT_EQ(9, countMeshElementsIf(nonCompressedMesh, isLine)); 
    EXPECT_EQ(0, countMeshElementsIf(nonCompressedMesh, isNode));

    EXPECT_EQ(1, countRepeatedElements(compressedMesh));
    EXPECT_EQ(3, compressedMesh.groups[0].elements.size());
    EXPECT_EQ(6, nonCompressedMesh.groups[1].elements.size());
    EXPECT_EQ(1, countMeshElementsIf(compressedMesh, isQuad));
    EXPECT_EQ(8, countMeshElementsIf(compressedMesh, isLine)); 
    EXPECT_EQ(0, countMeshElementsIf(compressedMesh, isNode));
}

TEST_F(StaircaseMesherTest, mesh_tetrahedron_volume_2x2){

	Mesh m = buildCubeVolumeMesh(0.5);
    meshlib::meshers::StaircaseMesherOptions opts;
    opts.volumeGroups.insert(0);

// #if APP_LOADED
//     vtkIO::exportMeshToVTU("testData/cases/mesh_tetrahedron_volume_2x2_before.vtk", m);
//     vtkIO::exportGridToVTU("testData/cases/mesh_tetrahedron_volume_2x2_before_grid.vtk", m.grid);
// #endif

    auto staircasedMesh = StaircaseMesher{m, 4, opts }.mesh();

// #if APP_LOADED
//     vtkIO::exportMeshToVTU("testData/cases/mesh_tetrahedron_volume_2x2_after.vtk", staircasedMesh);
//     vtkIO::exportGridToVTU("testData/cases/mesh_tetrahedron_volume_2x2_after_grid.vtk", staircasedMesh.grid);
// #endif

    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTriangle));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isQuad));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTetrahedron));
    EXPECT_EQ(4, countMeshElementsIf(staircasedMesh, isHexahedron));
    EXPECT_EQ(18, staircasedMesh.coordinates.size());

}

TEST_F(StaircaseMesherTest, mesh_surface_volume_2x2){

	Mesh m = buildCubeSurfaceMesh(0.5);
    meshlib::meshers::StaircaseMesherOptions opts;
    opts.volumeGroups.insert(0);
    // opts.isVolume = true;
    auto staircasedMesh = StaircaseMesher{m, 4, opts }.mesh();

    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTriangle));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isQuad));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTetrahedron));
    EXPECT_EQ(4, countMeshElementsIf(staircasedMesh, isHexahedron));
    EXPECT_EQ(18, staircasedMesh.coordinates.size());

}

TEST_F(StaircaseMesherTest, mesh_surface_not_volume_2x2){

	Mesh m = buildCubeSurfaceMesh(0.5);
    meshlib::meshers::StaircaseMesherOptions opts;
    opts.compress = false;
    // opts.isVolume = true;
    auto staircasedMesh = StaircaseMesher{m, 4, opts }.mesh();

    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTriangle));
    EXPECT_EQ(24, countMeshElementsIf(staircasedMesh, isQuad));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isTetrahedron));
    EXPECT_EQ(0, countMeshElementsIf(staircasedMesh, isHexahedron));

}

TEST_F(StaircaseMesherTest, meshesSelectedNonzeroVolumeGroupWithHexahedra)
{
    Mesh mesh = buildCubeSurfaceMesh(0.5);
    mesh.groups[0].name = "surface";
    mesh.groups.push_back(mesh.groups[0]);
    mesh.groups[1].name = "volume";
    StaircaseMesherOptions options;
    options.compress = false;
    options.volumeGroups.insert(1);

    const Mesh result = StaircaseMesher(mesh, 4, options).mesh();

    EXPECT_EQ("surface", result.groups[0].name);
    EXPECT_EQ("volume", result.groups[1].name);
    EXPECT_EQ(24, std::count_if(
        result.groups[0].elements.begin(), result.groups[0].elements.end(), isQuad));
    EXPECT_EQ(4, std::count_if(
        result.groups[1].elements.begin(), result.groups[1].elements.end(), isHexahedron));
}

#if APP_LOADED

TEST_F(StaircaseMesherTest, fillsSphereAsSingleClosedUnitHexahedralVolume)
{
    auto mesh = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    
    mesh.grid[X] = utils::GridTools::linspace(-100.0, 100.0, 51); 
    mesh.grid[Y] = utils::GridTools::linspace(-100.0, 100.0, 51); 
    mesh.grid[Z] = utils::GridTools::linspace(-100.0, 100.0, 51);

    ASSERT_TRUE(isSingleClosedSurface(mesh.groups[0].elements));

    StaircaseMesherOptions options;
    options.volumeGroups.insert(0);
    options.splitHexahedra = true;

    // vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.volume.before.vtk", mesh);
    const Mesh result = StaircaseMesher{mesh, 4, options}.mesh();
    // vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.volume.after.vtk", result);

    EXPECT_EQ(7967, countMeshElementsIf(result, isHexahedron));
    EXPECT_EQ(result.countElems(), countMeshElementsIf(result, isHexahedron));
    EXPECT_TRUE(isSingleClosedHexahedralVolume(result));
}

TEST_F(StaircaseMesherTest, fillsAlhambraAsSingleClosedUnitHexahedralVolume)
{
    auto mesh = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
    mesh.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61);
    mesh.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61);
    mesh.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    ASSERT_TRUE(isSingleClosedSurface(mesh.groups[0].elements));

    StaircaseMesherOptions options;
    options.volumeGroups.insert(0);
    options.splitHexahedra = true;

    // vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.volume.before.vtk", mesh);
    const Mesh result = StaircaseMesher{mesh, 4, options}.mesh();
    // vtkIO::exportMeshToVTU("testData/cases/alhambra/alhambra.volume.after.vtk", result);

    EXPECT_EQ(7255, countMeshElementsIf(result, isHexahedron));
    EXPECT_EQ(result.countElems(), countMeshElementsIf(result, isHexahedron));
    EXPECT_TRUE(isSingleClosedHexahedralVolume(result));

}

TEST_F(StaircaseMesherTest, preserves_topological_closedness_for_alhambra)
{
    auto mesh = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
    
    mesh.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    mesh.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    mesh.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    StaircaseMesherOptions options;
    options.compress = false;
    auto staircasedMesh = StaircaseMesher{mesh, 4, options}.mesh();
    
    EXPECT_TRUE(meshTools::isAClosedTopology(mesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(staircasedMesh.groups[0].elements));
}

TEST_F(StaircaseMesherTest, preserves_topological_closedness_for_sphere)
{
    auto mesh = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        mesh.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    StaircaseMesherOptions options;
    options.compress = false;
    auto staircasedMesh = StaircaseMesher{mesh, 4, options}.mesh();
    
    EXPECT_TRUE(meshTools::isAClosedTopology(mesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(staircasedMesh.groups[0].elements));

    // //For debugging.
	// meshTools::convertToAbsoluteCoordinates(staircasedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.sliced.vtk", staircasedMesh);

	// auto contourMesh = meshTools::buildMeshFromContours(staircasedMesh);
	// vtkIO::exportMeshToVTU("testData/cases/sphere/sphere.contour.vtk", contourMesh);
}

TEST_F(StaircaseMesherTest, selectiveStaircaser_preserves_topological_closedness_for_sphere)
{
    const std::string inputFilename = "testData/cases/sphere/sphere.stl";
    auto mesh = vtkIO::readInputMesh("testData/cases/sphere/sphere.stl");
    for (auto x: {X,Y,Z}) {
        mesh.grid[x] = utils::GridTools::linspace(-50.0, 50.0, 26); 
    }

    // SurfaceMesh

    auto surfaceMesh = meshlib::utils::meshTools::buildMeshFilteringElements(mesh, meshlib::utils::meshTools::isNotTetrahedron);

    // Slicer

    auto slicedMesh = meshlib::core::Slicer{surfaceMesh}.getMesh();

    // Collapser

    auto collapsedMesh = meshlib::core::Collapser{slicedMesh, 4}.getMesh();

    // Selection the specific cells to staircase and generate the result Mesh

    std::set<Cell> cellSet;

    for (int x = 0; x < 26; ++x) {
        for (int y = 0; y < 26; ++y) {
            for (int z = 0; z < 13; ++z) {  
                cellSet.insert(Cell{x, y, z});
            }
        }
    }

    meshlib::core::Staircaser staircaser{ collapsedMesh };

    auto resultMesh = staircaser.getSelectiveMesh(cellSet, meshlib::core::Staircaser::GapsFillingType::Insert);

    RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(resultMesh);
    utils::meshTools::reduceGrid(resultMesh, mesh.grid);
    utils::meshTools::convertToAbsoluteCoordinates(resultMesh);
    
    EXPECT_TRUE(meshTools::isAClosedTopology(mesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(resultMesh.groups[0].elements));   
    
    // // For debugging
    // std::filesystem::path outputFolder = meshlib::vtkIO::getFolder(inputFilename);
    // auto basename = meshlib::vtkIO::getBasename(inputFilename);
    // meshlib::vtkIO::exportMeshToVTU(outputFolder / (basename + ".tessellator.selective.vtk"), resultMesh);
    // meshlib::vtkIO::exportGridToVTU(outputFolder / (basename + ".tessellator.selective.grid.vtk"), resultMesh.grid);
}

TEST_F(StaircaseMesherTest, selectiveStaircaser_preserves_topological_closedness_for_alhambra)
{
    const std::string inputFilename = "testData/cases/alhambra/alhambra.stl";
    auto mesh = vtkIO::readInputMesh("testData/cases/alhambra/alhambra.stl");
    
    mesh.grid[X] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    mesh.grid[Y] = utils::GridTools::linspace(-60.0, 60.0, 61); 
    mesh.grid[Z] = utils::GridTools::linspace(-1.872734, 11.236404, 8);
    
    // SurfaceMesh

    auto surfaceMesh = meshlib::utils::meshTools::buildMeshFilteringElements(mesh, meshlib::utils::meshTools::isNotTetrahedron);

    // Slicer

    auto slicedMesh = meshlib::core::Slicer{surfaceMesh}.getMesh();

    // Collapser

    auto collapsedMesh = meshlib::core::Collapser{slicedMesh, 2}.getMesh();

    // Selection the specific cells to staircase and generate the result Mesh

    std::set<Cell> cellSet;

    for (int x = 0; x < 31; ++x) {
        for (int y = 0; y < 61; ++y) {
            for (int z = 0; z < 8; ++z) {  
                cellSet.insert(Cell{x, y, z});
            }
        }
    }

    auto resultMesh = meshlib::core::Staircaser{ collapsedMesh }.getSelectiveMesh(cellSet);

    RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(resultMesh);
    utils::meshTools::reduceGrid(resultMesh, mesh.grid);
    utils::meshTools::convertToAbsoluteCoordinates(resultMesh);
    
    EXPECT_TRUE(meshTools::isAClosedTopology(mesh.groups[0].elements));
    EXPECT_TRUE(meshTools::isAClosedTopology(resultMesh.groups[0].elements));

    // // For debugging
    // std::filesystem::path outputFolder = meshlib::vtkIO::getFolder(inputFilename);
    // auto basename = meshlib::vtkIO::getBasename(inputFilename);
    // meshlib::vtkIO::exportMeshToVTU(outputFolder / (basename + ".tessellator.selective.vtk"), resultMesh);
    // meshlib::vtkIO::exportGridToVTU(outputFolder / (basename + ".tessellator.selective.grid.vtk"), resultMesh.grid);
}

TEST_F(StaircaseMesherTest, staircaser_reads_wires_correctly)
{
    const std::string inputFilename = "testData/cases/longPolyline/longPolyline.vtu";
    auto mesh = vtkIO::readInputMesh("testData/cases/longPolyline/longPolyline.vtu");

    mesh.grid[X] = utils::GridTools::linspace(600, 1100.0, 20);
    mesh.grid[Y] = utils::GridTools::linspace(600, 1100.0, 20);
    mesh.grid[Z] = utils::GridTools::linspace(600, 1100.0, 20);

    std::vector<Element::Type> dimensions = { Element::Type::Line };

    // SurfaceMesh

    auto surfaceMesh = meshlib::utils::meshTools::buildMeshFilteringElements(mesh, meshlib::utils::meshTools::isNotTetrahedron);

    // Slicer

    auto slicedMesh = meshlib::core::Slicer{ surfaceMesh, dimensions }.getMesh();

    // Collapser

    auto collapsedMesh = meshlib::core::Collapser{ slicedMesh, 2, dimensions }.getMesh();

    // Selection the specific cells to staircase and generate the result Mesh

    std::set<Cell> cellSet;

    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            for (int z = 0; z < 20; ++z) {
                cellSet.insert(Cell{ x, y, z });
            }
        }
    }

    auto resultMesh = meshlib::core::Staircaser{ collapsedMesh }.getMesh();

    RedundancyCleaner::removeOverlappedElementsByDimension(resultMesh, dimensions);
    utils::meshTools::reduceGrid(resultMesh, mesh.grid);
    utils::meshTools::convertToAbsoluteCoordinates(resultMesh);

    assertMeshEqual(StaircaseMesher(mesh).mesh(), resultMesh);

    // // For debugging
    // std::filesystem::path outputFolder = meshlib::vtkIO::getFolder(inputFilename);
    // auto basename = meshlib::vtkIO::getBasename(inputFilename);
    // meshlib::vtkIO::exportMeshToVTU(outputFolder / (basename + ".tessellator.selective.vtk"), resultMesh);
    // meshlib::vtkIO::exportGridToVTU(outputFolder / (basename + ".tessellator.selective.grid.vtk"), resultMesh.grid);
}


#endif

}
