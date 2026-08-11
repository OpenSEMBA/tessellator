#include "gtest/gtest.h"

#include "MeshFixtures.h"
#include "core/VolumeShellExtractor.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

namespace meshlib::core {

using namespace meshFixtures;
using namespace utils::meshTools;

namespace {

double signedVolume6(const Mesh& mesh, const Group& group)
{
    double result = 0.0;
    for (const Element& element : group.elements) {
        const auto& first = mesh.coordinates[element.vertices[0]];
        const auto& second = mesh.coordinates[element.vertices[1]];
        const auto& third = mesh.coordinates[element.vertices[2]];
        result += first * (second ^ third);
    }
    return result;
}

Mesh buildOpenTetrahedronShell()
{
    Mesh mesh = buildTetSurfaceMesh(1.0);
    mesh.groups[0].elements.pop_back();
    return mesh;
}

}

TEST(VolumeShellExtractorTest, extractsOutwardBoundaryFromTetrahedron)
{
    const Mesh shell = VolumeShellExtractor(buildTetMesh(1.0)).getMesh();

    EXPECT_EQ(4, countMeshElementsIf(shell, isTriangle));
    EXPECT_TRUE(isAClosedTopology(shell.groups[0].elements));
    EXPECT_GT(signedVolume6(shell, shell.groups[0]), 0.0);
}

TEST(VolumeShellExtractorTest, removesFacesInsideConnectedTetrahedrons)
{
    const Mesh shell = VolumeShellExtractor(buildTetMeshWithInnerPoint(1.0)).getMesh();

    EXPECT_EQ(4, countMeshElementsIf(shell, isTriangle));
    EXPECT_EQ(4, shell.coordinates.size());
    EXPECT_TRUE(isAClosedTopology(shell.groups[0].elements));
}

TEST(VolumeShellExtractorTest, acceptsAndOrientsClosedTriangleShell)
{
    Mesh input = buildTetSurfaceMesh(1.0);
    std::reverse(
        input.groups[0].elements[0].vertices.begin(),
        input.groups[0].elements[0].vertices.end());

    const Mesh shell = VolumeShellExtractor(input).getMesh();

    EXPECT_EQ(4, shell.countElems());
    EXPECT_TRUE(isAClosedTopology(shell.groups[0].elements));
    EXPECT_GT(signedVolume6(shell, shell.groups[0]), 0.0);
}

TEST(VolumeShellExtractorTest, preservesEmptyGroupsAndNames)
{
    Mesh input = buildTetMesh(1.0);
    input.groups[0].name = "volume";
    input.groups.insert(input.groups.begin(), Group{"surface", {}});

    const Mesh shell = VolumeShellExtractor(input).getMesh();

    ASSERT_EQ(2, shell.groups.size());
    EXPECT_EQ("surface", shell.groups[0].name);
    EXPECT_TRUE(shell.groups[0].elements.empty());
    EXPECT_EQ("volume", shell.groups[1].name);
    EXPECT_EQ(4, shell.groups[1].elements.size());
}

TEST(VolumeShellExtractorTest, acceptsDisconnectedClosedComponents)
{
    Mesh input = buildTetMesh(1.0);
    input.coordinates.insert(input.coordinates.end(), {
        Coordinate({2.0, 0.0, 0.0}),
        Coordinate({3.0, 0.0, 0.0}),
        Coordinate({2.0, 1.0, 0.0}),
        Coordinate({2.0, 0.0, 1.0})});
    input.groups[0].elements.push_back(
        Element({4, 5, 6, 7}, Element::Type::Volume));

    const Mesh shell = VolumeShellExtractor(input).getMesh();

    EXPECT_EQ(8, shell.countElems());
    EXPECT_GT(signedVolume6(shell, shell.groups[0]), 0.0);
}

TEST(VolumeShellExtractorTest, rejectsOpenShell)
{
    EXPECT_THROW(VolumeShellExtractor{buildOpenTetrahedronShell()}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsMixedTrianglesAndTetrahedrons)
{
    Mesh input = buildTetMesh(1.0);
    input.groups[0].elements.push_back(Element({0, 1, 2}));

    EXPECT_THROW(VolumeShellExtractor{input}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsNonManifoldEdge)
{
    Mesh input = buildTetSurfaceMesh(1.0);
    input.coordinates.push_back(Coordinate({0.0, -1.0, 0.0}));
    input.groups[0].elements.push_back(Element({0, 1, 4}));

    EXPECT_THROW(VolumeShellExtractor{input}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsTetrahedronsTouchingOnlyAtOneVertex)
{
    Mesh input = buildTetMesh(1.0);
    input.coordinates.insert(input.coordinates.end(), {
        Coordinate({-1.0, 0.0, 0.0}),
        Coordinate({0.0, -1.0, 0.0}),
        Coordinate({0.0, 0.0, -1.0})});
    input.groups[0].elements.push_back(
        Element({0, 4, 5, 6}, Element::Type::Volume));

    EXPECT_THROW(VolumeShellExtractor{input}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsTetrahedronsTouchingOnlyAtOneEdge)
{
    EXPECT_THROW(
        VolumeShellExtractor{buildTetsSharingEdgeMesh()},
        std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsFaceSharedByThreeTetrahedrons)
{
    Mesh input = buildTetMesh(1.0);
    input.coordinates.push_back(Coordinate({0.0, 0.0, -1.0}));
    input.coordinates.push_back(Coordinate({0.2, 0.2, -1.0}));
    input.groups[0].elements.push_back(
        Element({0, 2, 1, 4}, Element::Type::Volume));
    input.groups[0].elements.push_back(
        Element({0, 1, 2, 5}, Element::Type::Volume));

    EXPECT_THROW(VolumeShellExtractor{input}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsInvalidCoordinateId)
{
    Mesh input = buildTetMesh(1.0);
    input.groups[0].elements[0].vertices[3] = input.coordinates.size();

    EXPECT_THROW(VolumeShellExtractor{input}, std::runtime_error);
}

TEST(VolumeShellExtractorTest, rejectsDegenerateAndUnsupportedElements)
{
    Mesh degenerate = buildTetMesh(1.0);
    degenerate.coordinates[3] = Coordinate({0.5, 0.5, 0.0});
    EXPECT_THROW(VolumeShellExtractor{degenerate}, std::runtime_error);

    Mesh unsupported = buildTetMesh(1.0);
    unsupported.groups[0].elements = {
        Element({0, 1, 2, 3, 0, 1, 2, 3}, Element::Type::Volume)};
    EXPECT_THROW(VolumeShellExtractor{unsupported}, std::runtime_error);
}

}
