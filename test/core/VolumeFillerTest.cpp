#include "gtest/gtest.h"

#include "core/VolumeFiller.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"

namespace meshlib::core {

using namespace utils;
using namespace meshTools;

namespace {

CoordinateId coordinateId(Mesh& mesh, CellDir x, CellDir y, CellDir z)
{
    const Coordinate coordinate = GridTools(mesh.grid).getPos(Cell({x, y, z}));
    const auto found = std::find(mesh.coordinates.begin(), mesh.coordinates.end(), coordinate);
    if (found != mesh.coordinates.end()) {
        return found - mesh.coordinates.begin();
    }
    mesh.coordinates.push_back(coordinate);
    return mesh.coordinates.size() - 1;
}

void addQuad(Mesh& mesh, const std::array<Cell, 4>& cells)
{
    Element quad;
    quad.type = Element::Type::Surface;
    for (const auto& cell : cells) {
        quad.vertices.push_back(coordinateId(mesh, cell[X], cell[Y], cell[Z]));
    }
    mesh.groups[0].elements.push_back(quad);
}

Mesh buildTwoByTwoByTwoShell()
{
    Mesh mesh;
    mesh.grid = GridTools::buildCartesianGrid(0.0, 1.0, 3);
    mesh.groups.resize(1);
    mesh.groups[0].name = "volume";

    for (CellDir first = 0; first < 2; ++first) {
        for (CellDir second = 0; second < 2; ++second) {
            for (CellDir x : {CellDir(0), CellDir(2)}) {
                addQuad(mesh, {
                    Cell({x, first, second}), Cell({x, first + 1, second}),
                    Cell({x, first + 1, second + 1}), Cell({x, first, second + 1})});
            }
            for (CellDir y : {CellDir(0), CellDir(2)}) {
                addQuad(mesh, {
                    Cell({first, y, second}), Cell({first + 1, y, second}),
                    Cell({first + 1, y, second + 1}), Cell({first, y, second + 1})});
            }
            for (CellDir z : {CellDir(0), CellDir(2)}) {
                addQuad(mesh, {
                    Cell({first, second, z}), Cell({first + 1, second, z}),
                    Cell({first + 1, second + 1, z}), Cell({first, second + 1, z})});
            }
        }
    }
    return mesh;
}

void addBox(Mesh& mesh, CellDir x0, CellDir x1)
{
    addQuad(mesh, {Cell({x0, 0, 0}), Cell({x0, 1, 0}),
                   Cell({x0, 1, 1}), Cell({x0, 0, 1})});
    addQuad(mesh, {Cell({x1, 0, 0}), Cell({x1, 1, 0}),
                   Cell({x1, 1, 1}), Cell({x1, 0, 1})});
    addQuad(mesh, {Cell({x0, 0, 0}), Cell({x1, 0, 0}),
                   Cell({x1, 0, 1}), Cell({x0, 0, 1})});
    addQuad(mesh, {Cell({x0, 1, 0}), Cell({x1, 1, 0}),
                   Cell({x1, 1, 1}), Cell({x0, 1, 1})});
    addQuad(mesh, {Cell({x0, 0, 0}), Cell({x1, 0, 0}),
                   Cell({x1, 1, 0}), Cell({x0, 1, 0})});
    addQuad(mesh, {Cell({x0, 0, 1}), Cell({x1, 0, 1}),
                   Cell({x1, 1, 1}), Cell({x0, 1, 1})});
}

}

TEST(VolumeFillerTest, fillsContinuousRunsWithHexahedra)
{
    const Mesh result = VolumeFiller(buildTwoByTwoByTwoShell()).getMesh();

    EXPECT_EQ("volume", result.groups[0].name);
    EXPECT_EQ(4, countMeshElementsIf(result, isHexahedron));
    EXPECT_EQ(0, countMeshElementsIf(result, isQuad));
    EXPECT_EQ(18, result.coordinates.size());

    const auto& first = result.groups[0].elements.front();
    const Coordinates expected = {
        Coordinate({0.0, 0.0, 0.0}), Coordinate({1.0, 0.0, 0.0}),
        Coordinate({1.0, 0.5, 0.0}), Coordinate({0.0, 0.5, 0.0}),
        Coordinate({0.0, 0.0, 0.5}), Coordinate({1.0, 0.0, 0.5}),
        Coordinate({1.0, 0.5, 0.5}), Coordinate({0.0, 0.5, 0.5})};
    for (std::size_t vertex = 0; vertex < expected.size(); ++vertex) {
        EXPECT_EQ(expected[vertex], result.coordinates[first.vertices[vertex]]);
    }
}

TEST(VolumeFillerTest, splitsContinuousRunsIntoUnitCellHexahedra)
{
    const Mesh result = VolumeFiller(buildTwoByTwoByTwoShell(), true).getMesh();

    EXPECT_EQ(8, countMeshElementsIf(result, isHexahedron));
    const GridTools tools(result.grid);
    for (const auto& element : result.groups[0].elements) {
        ASSERT_TRUE(element.isHexahedron());
        Cell lower = tools.getCell(result.coordinates[element.vertices[0]]);
        Cell upper = lower;
        for (CoordinateId vertex : element.vertices) {
            const Cell cell = tools.getCell(result.coordinates[vertex]);
            for (Axis axis : {X, Y, Z}) {
                lower[axis] = std::min(lower[axis], cell[axis]);
                upper[axis] = std::max(upper[axis], cell[axis]);
            }
        }
        for (Axis axis : {X, Y, Z}) {
            EXPECT_EQ(1, upper[axis] - lower[axis]);
        }
    }
}

TEST(VolumeFillerTest, rejectsAnOpenQuadShell)
{
    Mesh shell = buildTwoByTwoByTwoShell();
    shell.groups[0].elements.erase(shell.groups[0].elements.begin());

    EXPECT_THROW(VolumeFiller{shell}, std::runtime_error);
}

TEST(VolumeFillerTest, fillsDisconnectedIntervalsOnTheSameRay)
{
    Mesh shell;
    shell.grid = GridTools::buildCartesianGrid(0.0, 3.0, 4);
    shell.groups.resize(1);
    addBox(shell, 0, 1);
    addBox(shell, 2, 3);

    const Mesh result = VolumeFiller(shell).getMesh();

    ASSERT_EQ(2, result.countElems());
    EXPECT_TRUE(result.groups[0].elements[0].isHexahedron());
    EXPECT_TRUE(result.groups[0].elements[1].isHexahedron());
}

}
