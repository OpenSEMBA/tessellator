#include "gtest/gtest.h"

#include "core/ThinSlotBuilder.h"

#include "utils/GridTools.h"
#include "utils/MeshTools.h"

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <stdexcept>

namespace meshlib::core {

class ThinSlotBuilderTest : public ::testing::Test {
};

namespace {

Coordinate coordinate(int x, int y, int z)
{
    return Coordinate({ static_cast<double>(x), static_cast<double>(y), static_cast<double>(z) });
}

CoordinateId addCoordinate(Mesh& mesh, std::map<Cell, CoordinateId>& ids, const Cell& cell)
{
    const auto existing = ids.find(cell);
    if (existing != ids.end()) {
        return existing->second;
    }
    const CoordinateId id = mesh.coordinates.size();
    mesh.coordinates.push_back(coordinate(cell[0], cell[1], cell[2]));
    ids.emplace(cell, id);
    return id;
}

Mesh makeSurface(const std::vector<std::array<Cell, 4>>& quads)
{
    Mesh mesh;
    mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
    mesh.groups.resize(1);
    mesh.groups[0].name = "surface";
    std::map<Cell, CoordinateId> ids;
    for (const auto& quad : quads) {
        Element element;
        element.type = Element::Type::Surface;
        for (const Cell& cell : quad) {
            element.vertices.push_back(addCoordinate(mesh, ids, cell));
        }
        mesh.groups[0].elements.push_back(std::move(element));
    }
    return mesh;
}

Mesh makeLine(const std::vector<std::array<Cell, 2>>& segments)
{
    Mesh mesh;
    mesh.grid = utils::GridTools::buildCartesianGrid(0.0, 4.0, 5);
    mesh.groups.resize(1);
    mesh.groups[0].name = "line";
    std::map<Cell, CoordinateId> ids;
    for (const auto& segment : segments) {
        Element element;
        element.type = Element::Type::Line;
        for (const Cell& cell : segment) {
            element.vertices.push_back(addCoordinate(mesh, ids, cell));
        }
        mesh.groups[0].elements.push_back(std::move(element));
    }
    return mesh;
}

std::array<Cell, 4> unitQuad(int x, int y)
{
    return { {
        Cell({ x, y, 0 }),
        Cell({ x + 1, y, 0 }),
        Cell({ x + 1, y + 1, 0 }),
        Cell({ x, y + 1, 0 })
    } };
}

std::vector<std::array<Cell, 4>> unitSurface(int columns, int rows)
{
    std::vector<std::array<Cell, 4>> quads;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            quads.push_back(unitQuad(x, y));
        }
    }
    return quads;
}

std::size_t countQuads(const Mesh& mesh)
{
    return utils::meshTools::countMeshElementsIf(
        mesh, [](const Element& element) { return element.isQuad(); });
}

std::size_t maximumSelectedQuadDegree(const Mesh& mesh)
{
    std::vector<const Element*> quads;
    for (const Group& group : mesh.groups) {
        for (const Element& element : group.elements) {
            if (element.isQuad()) {
                quads.push_back(&element);
            }
        }
    }

    std::map<std::pair<CoordinateId, CoordinateId>, std::vector<std::size_t>> edgeQuads;
    for (std::size_t quadId = 0; quadId < quads.size(); ++quadId) {
        for (std::size_t vertex = 0; vertex < 4; ++vertex) {
            CoordinateId first = quads[quadId]->vertices[vertex];
            CoordinateId second = quads[quadId]->vertices[(vertex + 1) % 4];
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

    std::size_t maximum = 0;
    for (const auto& neighbours : adjacency) {
        maximum = std::max(maximum, neighbours.size());
    }
    return maximum;
}

std::size_t countSelectedQuadsAtVertex(const Mesh& mesh)
{
    std::map<std::array<double, 3>, std::size_t> uses;
    std::size_t maximum = 0;
    for (const Group& group : mesh.groups) {
        for (const Element& element : group.elements) {
            if (!element.isQuad()) {
                continue;
            }
            for (const CoordinateId vertex : element.vertices) {
                const Coordinate& coordinate = mesh.coordinates[vertex];
                const std::array<double, 3> key{
                    coordinate[0], coordinate[1], coordinate[2] };
                maximum = std::max(maximum, ++uses[key]);
            }
        }
    }
    return maximum;
}

}

TEST_F(ThinSlotBuilderTest, expandsCompressedQuadAndEmbedsInteriorLine)
{
    const Mesh host = makeSurface({ {
        Cell({ 0, 0, 0 }),
        Cell({ 3, 0, 0 }),
        Cell({ 3, 1, 0 }),
        Cell({ 0, 1, 0 })
    } });
    const Mesh line = makeLine({ {
        Cell({ 1, 0, 0 }),
        Cell({ 1, 1, 0 })
    } });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    ASSERT_EQ(1u, components.size());
    ASSERT_EQ(1u, components.front().size());

    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);
    EXPECT_EQ(3u, countQuads(cellAligned));
    EXPECT_NO_THROW(ThinSlotBuilder::validateEmbedding(cellAligned, components, false));

    const Mesh strip = ThinSlotBuilder::buildSurfaceStrip(cellAligned, components);
    EXPECT_EQ(1u, countQuads(strip));
}

TEST_F(ThinSlotBuilderTest, rejectsPerimeterUnlessAllowed)
{
    const Mesh host = makeSurface({ {
        Cell({ 0, 0, 0 }),
        Cell({ 3, 0, 0 }),
        Cell({ 3, 1, 0 }),
        Cell({ 0, 1, 0 })
    } });
    const Mesh line = makeLine({ {
        Cell({ 0, 0, 0 }),
        Cell({ 0, 1, 0 })
    } });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);

    EXPECT_THROW(
        ThinSlotBuilder::validateEmbedding(cellAligned, components, false),
        std::runtime_error);
    EXPECT_NO_THROW(
        ThinSlotBuilder::validateEmbedding(cellAligned, components, true));
}

TEST_F(ThinSlotBuilderTest, rejectsLineOutsideHostSurface)
{
    const Mesh host = makeSurface({ unitQuad(0, 0) });
    const Mesh line = makeLine({ {
        Cell({ 0, 0, 1 }),
        Cell({ 1, 0, 1 })
    } });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);

    EXPECT_THROW(
        ThinSlotBuilder::validateEmbedding(cellAligned, components, true),
        std::runtime_error);
}

TEST_F(ThinSlotBuilderTest, ordersUnorderedLineSegmentsIntoOnePath)
{
    const Mesh line = makeLine({
        { Cell({ 1, 1, 0 }), Cell({ 2, 1, 0 }) },
        { Cell({ 0, 1, 0 }), Cell({ 1, 1, 0 }) },
        { Cell({ 2, 1, 0 }), Cell({ 3, 1, 0 }) }
    });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    ASSERT_EQ(1u, components.size());
    ASSERT_EQ(3u, components.front().size());
    EXPECT_EQ(Cell({ 0, 1, 0 }), components.front().front()[0]);
    EXPECT_EQ(Cell({ 3, 1, 0 }), components.front().back()[1]);
}

TEST_F(ThinSlotBuilderTest, splitsCompressedLineIntoUnitSegments)
{
    const Mesh line = makeLine({ {
        Cell({ 0, 0, 0 }),
        Cell({ 3, 0, 0 })
    } });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    ASSERT_EQ(1u, components.size());
    EXPECT_EQ(3u, components.front().size());
}

TEST_F(ThinSlotBuilderTest, rejectsDiagonalLine)
{
    const Mesh line = makeLine({ {
        Cell({ 0, 0, 0 }),
        Cell({ 1, 1, 0 })
    } });

    EXPECT_THROW(
        ThinSlotBuilder::buildOrderedUnitSegments(line),
        std::runtime_error);
}

TEST_F(ThinSlotBuilderTest, buildsOneQuadWideStripAlongInternalEdge)
{
    const Mesh host = makeSurface(unitSurface(3, 3));
    const Mesh line = makeLine({
        { Cell({ 1, 0, 0 }), Cell({ 1, 1, 0 }) },
        { Cell({ 1, 1, 0 }), Cell({ 1, 2, 0 }) },
        { Cell({ 1, 2, 0 }), Cell({ 1, 3, 0 }) }
    });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);
    EXPECT_NO_THROW(ThinSlotBuilder::validateEmbedding(cellAligned, components, false));

    const Mesh strip = ThinSlotBuilder::buildSurfaceStrip(cellAligned, components);
    EXPECT_EQ(3u, countQuads(strip));
    EXPECT_LE(countSelectedQuadsAtVertex(strip), 3u);
    EXPECT_LE(maximumSelectedQuadDegree(strip), 2u);
}

TEST_F(ThinSlotBuilderTest, buildsContinuousStripAroundTurn)
{
    const Mesh host = makeSurface(unitSurface(3, 3));
    const Mesh line = makeLine({
        { Cell({ 1, 0, 0 }), Cell({ 1, 1, 0 }) },
        { Cell({ 1, 1, 0 }), Cell({ 2, 1, 0 }) }
    });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);
    EXPECT_NO_THROW(ThinSlotBuilder::validateEmbedding(cellAligned, components, false));

    const Mesh strip = ThinSlotBuilder::buildSurfaceStrip(cellAligned, components);
    EXPECT_GT(countQuads(strip), 0u);
    EXPECT_LE(countSelectedQuadsAtVertex(strip), 3u);
    EXPECT_LE(maximumSelectedQuadDegree(strip), 2u);
}

TEST_F(ThinSlotBuilderTest, deduplicatesSameDirectionSegments)
{
    const Mesh line = makeLine({
        { Cell({ 0, 0, 0 }), Cell({ 1, 0, 0 }) },
        { Cell({ 0, 0, 0 }), Cell({ 1, 0, 0 }) },
        { Cell({ 0, 0, 0 }), Cell({ 1, 0, 0 }) }
    });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    ASSERT_EQ(1u, components.size());
    EXPECT_EQ(1u, components.front().size());
    EXPECT_EQ(Cell({ 0, 0, 0 }), components.front().front()[0]);
    EXPECT_EQ(Cell({ 1, 0, 0 }), components.front().front()[1]);
}

TEST_F(ThinSlotBuilderTest, cancelsOppositeSegments)
{
    const Mesh line = makeLine({
        { Cell({ 0, 0, 0 }), Cell({ 1, 0, 0 }) },
        { Cell({ 1, 0, 0 }), Cell({ 0, 0, 0 }) }
    });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    EXPECT_TRUE(components.empty());
}

TEST_F(ThinSlotBuilderTest, overlappingHostFacesDoNotHideSurfacePerimeter)
{
    const Mesh host = makeSurface({
        unitQuad(0, 0),
        unitQuad(0, 0)
    });
    const Mesh line = makeLine({ {
        Cell({ 0, 0, 0 }),
        Cell({ 0, 1, 0 })
    } });

    const auto components = ThinSlotBuilder::buildOrderedUnitSegments(line);
    const Mesh cellAligned = ThinSlotBuilder::buildCellAlignedSurface(host, components);

    EXPECT_THROW(
        ThinSlotBuilder::validateEmbedding(cellAligned, components, false),
        std::runtime_error);
    EXPECT_NO_THROW(
        ThinSlotBuilder::validateEmbedding(cellAligned, components, true));
}

TEST_F(ThinSlotBuilderTest, rejectsBranchingLine)
{
    const Mesh line = makeLine({
        { Cell({ 1, 1, 0 }), Cell({ 1, 0, 0 }) },
        { Cell({ 1, 1, 0 }), Cell({ 2, 1, 0 }) },
        { Cell({ 1, 1, 0 }), Cell({ 0, 1, 0 }) }
    });

    EXPECT_THROW(
        ThinSlotBuilder::buildOrderedUnitSegments(line),
        std::runtime_error);
}

}
