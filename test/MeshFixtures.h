#include "types/Mesh.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"

namespace meshlib {
namespace meshFixtures {

static Grid buildUnitLengthGrid(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    return utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);
}

static Mesh buildNonManifoldPatchMesh(double stepSize)
{
    // 2
    // | \
	// |  3
    // | / \
	// 0 -- {1,4}
    Mesh m;
    m.grid = meshFixtures::buildUnitLengthGrid(1.0);
    m.coordinates = {
        Coordinate({0.00, 0.00, 0.50}),
        Coordinate({1.00, 0.00, 0.00}),
        Coordinate({0.00, 1.00, 0.50}),
        Coordinate({0.50, 0.50, 0.50}),
        Coordinate({1.00, 0.00, 1.00}),
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 3, 1}),
        Element({3, 0, 2}),
        Element({0, 3, 4})
    };
    return m;
}

static Mesh buildTetAndTriMesh(double stepSize) 
{
    Mesh m;
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    m.grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    m.coordinates = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 }),
        Coordinate({ 1.0, 1.0, 1.0 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2, 3}, Element::Type::Volume),
        Element({1, 2, 4},    Element::Type::Surface)
    };
    
    return m;
}

static Mesh buildTetMesh(double stepSize) 
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 })
    };

    Element tet;
    tet.type = Element::Type::Volume;
    std::vector<Group> groups = { Group({std::vector<Element>(1, tet)}) };
    groups[0].elements[0].vertices = { 0, 1, 2, 3 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildTetSurfaceMesh(double stepSize) 
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 })
    };

    std::vector<Group> groups = { Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 2, 3}),
        Element({2, 3, 0}),
        Element({3, 0, 1})
    };

    return Mesh{ grid, coords, groups };
}

static Mesh buildTetMeshWithInnerPoint(double stepSize) {
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 }),
        Coordinate({ 0.1, 0.1, 0.1 })
    };

    Element tet;
    tet.type = Element::Type::Volume;
    std::vector<Group> groups = { Group({std::vector<Element>(4, tet)}) };
    groups[0].elements[0].vertices = { 0, 1, 2, 4 };
    groups[0].elements[1].vertices = { 0, 2, 3, 4 };
    groups[0].elements[2].vertices = { 0, 3, 1, 4 };
    groups[0].elements[3].vertices = { 1, 2, 3, 4 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildTetsSharingEdgeMesh() 
{
    Mesh m;
    
    const double boxMin = -61.0;
    const double boxMax = 64.0;
    const double stepSize = 5.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    m.grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    m.coordinates = {
        Coordinate({+4.86485365e+01, +4.15757576e+00, +1.00000000e-01}),
        Coordinate({+5.10000000e+01, +4.45454545e+00, +0.00000000e+00}),
        Coordinate({+5.10000000e+01, +8.90909091e+00, +5.00000000e-01}),
        Coordinate({+4.86485365e+01, +1.30666667e+01, +1.00000000e-01}),
        Coordinate({+5.10000000e+01, +1.33636364e+01, +0.00000000e+00}),
        Coordinate({+4.51140002e+01, +1.35823746e+01, +0.00000000e+00})
    };

    m.groups = {
        Group()
    },
    m.groups[0].elements = {
            Element({4, 3, 2, 5}, Element::Type::Volume),
            Element({1, 2, 0, 5}, Element::Type::Volume)
    };

    return m;
}

static Mesh buildTri45Mesh(double stepSize) 
{
    const double boxMin = 0.0;
    const double boxMax = 3.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    Coordinates coords{
        Coordinate({ 1.50000000e+00, 1.00000000e+00, 1.50000000e+00 }),
        Coordinate({ 1.00000000e+00, 2.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 1.00000000e+00, 1.00000000e+00 })
    };

    Groups groups{ Group{} };
    groups[0].elements = { Element{ {2, 0, 1} } };

    return Mesh{ grid, coords, groups };
}

static Mesh build3RegionsTSTMesh() 
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)1) + 1;
    auto grid{ utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum) };

    Coordinates coords{
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 1.0, 0.2, 0.0 }),
        Coordinate({ 1.0, 0.7, 0.0 }),
        Coordinate({ 1.0, 0.8, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 1.0 }),
        Coordinate({ 0.5, 0.2, 0.0 })
    };

    Groups groups{ Group{}, Group{} };
    groups[0].elements = { 
        Element{ {0, 1, 5} , Element::Type::Surface}, 
        Element{ {0, 2, 5} , Element::Type::Surface},
        Element{ {0, 6, 5} , Element::Type::Surface}
    };
    groups[1].elements = { 
        Element{ {0, 3, 4} , Element::Type::Surface} 
    };

    return Mesh{ grid, coords, groups };

}
static Mesh buildSingleCellTriMesh(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    auto grid{ utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum) };

    Coordinates coords{
        Coordinate({ 1.0, 0.2, 0.0 }),
        Coordinate({ 1.0, 0.2, 1.0 }),
        Coordinate({ 0.2, 1.0, 1.0 })
    };

    Groups groups{ Group{} };
    groups[0].elements = { Element{ {0, 2, 1} } };

    return Mesh{ grid, coords, groups };
}

static Mesh buildTriNonUniformGridMesh()
{
    Mesh m;
    m.grid;
    m.grid[0] = { -20.4645274743604, -20.420584, -20.332954, -20.2791060011645, -20.21205, -20.129754, -20.05584 };
    m.grid[1] = { 62.455298, 62.5094, 62.567058, 62.627256, 62.712092 };
    m.grid[2] = { 5.88009999999993E-02, 6.42619999999995E-02, 7.30249999999998E-02 };
    
    m.coordinates = {
        Coordinate({  -2.00690480e+01, +6.26435120e+01, +6.42620000e-02}),
        Coordinate({  -2.02953446e+01, +6.26506414e+01, +6.42620000e-02}),
        Coordinate({  -2.04252323e+01, +6.24846508e+01, +6.42620027e-02})
    };

    m.groups = { Group() };
    m.groups[0].elements = Elements({
        Element({1, 2, 0}, Element::Type::Surface)
        });

    return m;
}

static Mesh buildTriPartiallyOutOfGridMesh(double stepSize) 
{
    Mesh m;
    {
        const double bbMin = 0.0;
        const double bbMax = 2.0;
        const std::size_t gridLinesNum = (std::size_t)((bbMax - bbMin) / stepSize) + 1;
        m.grid = utils::GridTools::buildCartesianGrid(bbMin, bbMax, gridLinesNum);
    }

    m.coordinates = {
        Coordinate({   1.99, 1.99, 0.5 }),
        Coordinate({   1.99, 0.01, 0.5 }),
        Coordinate({ -10.00, 1.00, 0.5 })
    };

    m.groups = {Group()};
    m.groups[0].elements = Elements({
        Element({0, 1, 2}, Element::Type::Surface)
    });
    
    return m;
}

static Mesh buildTriOutOfGridMesh()
{
    Mesh m;
    {
        const double bbMin = 0.0;
        const double bbMax = 2.0;
        const std::size_t gridLinesNum = (std::size_t)((bbMax - bbMin) / 1.0) + 1;
        m.grid = utils::GridTools::buildCartesianGrid(bbMin, bbMax, gridLinesNum);
    }

    m.coordinates = {
        Coordinate({100.0, 100.0,  100.0}),
        Coordinate({ 90.0,  80.0,   90.0}),
        Coordinate({ 70.0,  70.0,   70.0}),
    };

    m.groups = { Group() };
    m.groups[0].elements = Elements({
        Element({0, 1, 2}, Element::Type::Surface)
        });

    return m;
}

static Mesh buildTri45Mesh2(double stepSize) 
{
    Mesh m;
    const double boxMin = 0.0;
    const double boxMax = 3.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    
    m.grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    m.coordinates = {
        Coordinate({ 3.00, 0.00, 0.50 }),
        Coordinate({ 0.00, 3.00, 1.00 }),
        Coordinate({ 0.00, 3.00, 0.00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = {
        Element({0, 1, 2})
    };

    return m;
}

static Mesh buildTwoSquaresXYMesh(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 0.4, 0.0, 0.0 }),
        Coordinate({ 0.0, 0.4, 0.0 }),
        Coordinate({ 0.4, 0.4, 0.0 }),
        Coordinate({ 0.6, 0.6, 0.0 }),
        Coordinate({ 1.0, 0.6, 0.0 }),
        Coordinate({ 0.6, 1.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 })
    };

    std::vector<Group> groups = { Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 3, 2}),
        Element({4, 5, 6}),
        Element({5, 7, 6})
    };
    return Mesh{ grid, coords, groups };
}

static Mesh buildTwoSquaresTwoGroupsXYMesh(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 0.4, 0.0, 0.0 }),
        Coordinate({ 0.0, 0.4, 0.0 }),
        Coordinate({ 0.4, 0.4, 0.0 }),
        Coordinate({ 0.6, 0.6, 0.0 }),
        Coordinate({ 1.0, 0.6, 0.0 }),
        Coordinate({ 0.6, 1.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 })
    };

    std::vector<Group> groups = { Group(), Group()};
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 3, 2})
    };
    groups[1].elements = {
        Element({4, 5, 6}),
        Element({5, 7, 6})
    };
    return Mesh{ grid, coords, groups };
}

static Mesh buildOverlappingTwoGroupsXYMesh(double stepSize)
{
    //
    // *--- 6-------7
    // |    |       |
    // 2----|----3  |
    // |    |   |   |
    // |    4---|---5
    // |        |   |
    // 0--------1---*
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 0.6, 0.0, 0.0 }),
        Coordinate({ 0.0, 0.6, 0.0 }),
        Coordinate({ 0.6, 0.6, 0.0 }),
        Coordinate({ 0.4, 0.4, 0.0 }),
        Coordinate({ 1.0, 0.4, 0.0 }),
        Coordinate({ 0.4, 1.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 })
    };

    std::vector<Group> groups = { Group(), Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 3, 2}) 
    };
    groups[1].elements = {
        Element({4, 5, 6}),
        Element({5, 7, 6})
    };

    return Mesh{ grid, coords, groups };
}

static Mesh buildOverlappingTwoGroupsXYMesh2(double stepSize)
{
    // *------------*
    // |            |
    // |            |
    // 2----6---3---7
    // |    |   |   |
    // |    |   |   |
    // |    |   |   |
    // 0----4---1---5
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 0.6, 0.0, 0.0 }),
        Coordinate({ 0.0, 0.6, 0.0 }),
        Coordinate({ 0.6, 0.6, 0.0 }),
        Coordinate({ 0.4, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.4, 0.6, 0.0 }),
        Coordinate({ 1.0, 0.6, 0.0 })
    };

    std::vector<Group> groups = { Group(), Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 3, 2})
    };
    groups[1].elements = {
        Element({4, 5, 6}),
        Element({5, 7, 6})
    };

    return Mesh{ grid, coords, groups };
}

static Mesh buildOverlappingTwoGroupsXYMeshNotSimple(double stepSize)
{
    // 3-------2
    // | \XXX/ |
    // |   4   |
    // | /XXX\ |
    // 0-------1
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 0.5, 0.5, 0.0 })
    };

    std::vector<Group> groups = { Group(), Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({2, 3, 0})
    };
    groups[1].elements = {
        Element({0, 1, 4}),
        Element({2, 3, 4})
    };

    return Mesh{ grid, coords, groups };
}

static Mesh buildOneTriangleOnXZ(double stepSize)
{
    // 2------ *
    // |\_     |
    // |   \_  |
    // |     \ |
    // 0-------1
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({0.0, 0.0, 0.0}),
        Coordinate({1.0, 0.0, 0.0}),
        Coordinate({0.0, 0.0, 1.0})
    };

    std::vector<Group> groups = { Group() };
    groups[0].elements = {
        Element({0, 1, 2 })
    };
    return Mesh{ grid, coords, groups };
}

static Mesh buildSelfOverlappingMesh(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.00, 0.00, 0.00 }),
        Coordinate({ 1.00, 0.00, 0.00 }),
        Coordinate({ 0.00, 1.00, 0.00 }),
        Coordinate({ 0.25, 0.25, 0.00 })
    };

    std::vector<Group> groups = { Group() };
    groups[0].elements = {
        Element({0, 1, 2}),
        Element({1, 0, 3})
    };
    return Mesh{ grid, coords, groups };
}

static Mesh buildPlaneXYMesh(double stepSize) {
    const double boxMin = 0.0;
    const double boxMax = 3.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 2.00000000e+00, 2.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.50000000e+00, 2.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 2.00000000e+00, 1.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.50000000e+00, 1.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 2.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 1.00000000e+00, 1.00000000e+00 })
    };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(4, tri)}) };
    groups[0].elements[0].vertices = { 5, 3, 4 };
    groups[0].elements[1].vertices = { 3, 1, 4 };
    groups[0].elements[2].vertices = { 3, 2, 1 };
    groups[0].elements[3].vertices = { 2, 0, 1 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildFrameXYMesh(double stepSize)
{
    const double boxMin = 0.0;
    const double boxMax = 1.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    auto grid{ utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum) };

    Coordinates coords{
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),

        Coordinate({ 0.2, 0.2, 0.0 }),
        Coordinate({ 0.8, 0.2, 0.0 }),
        Coordinate({ 0.8, 0.8, 0.0 }),
        Coordinate({ 0.2, 0.8, 0.0 })
    };

    Groups groups{ Group() };
    groups[0].elements = {
        Element({0, 4, 7}),
        Element({0, 1, 4}),
        Element({1, 5, 4}),
        Element({1, 6, 5}),
        Element({1, 2, 6}),
        Element({2, 3, 6}),
        Element({3, 7, 6}),
        Element({3, 0, 7})
    };

    return Mesh{ grid, coords, groups };
}

static Mesh buildPlaneXYTwoMaterialsMesh(double stepSize) {
    Mesh m{ buildPlaneXYMesh(stepSize) };
    
    m.groups.push_back(Group());
    Element tri;
    tri.type = Element::Type::Surface;
    m.groups[1] = Group({ std::vector<Element>(3, tri) });
    m.groups[1].elements[0].vertices = {5, 3, 1};
    m.groups[1].elements[1].vertices = {3, 0, 1};
    m.groups[1].elements[2].vertices = {3, 2, 1};

    m.groups[0].elements.pop_back();
    
    return m;
}

static Mesh buildPlane45Mesh(double stepSize) {
    const double boxMin = 0.0;
    const double boxMax = 3.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 2.00000000e+00, 2.00000000e+00, 2.00000000e+00 }),
        Coordinate({ 1.50000000e+00, 2.00000000e+00, 1.50000000e+00 }),
        Coordinate({ 2.00000000e+00, 1.00000000e+00, 2.00000000e+00 }),
        Coordinate({ 1.50000000e+00, 1.00000000e+00, 1.50000000e+00 }),
        Coordinate({ 1.00000000e+00, 2.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 1.00000000e+00, 1.00000000e+00 })
    };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(4, tri)}) };
    groups[0].elements[0].vertices = { 5, 3, 4 };
    groups[0].elements[1].vertices = { 3, 1, 4 };
    groups[0].elements[2].vertices = { 3, 2, 1 };
    groups[0].elements[3].vertices = { 2, 0, 1 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildPlane45TwoMaterialsMesh(double stepSize) 
{
    Mesh m = buildPlane45Mesh(stepSize);

    m.groups.push_back(Group());
    m.groups[1].elements.push_back(m.groups[0].elements[2]);
    m.groups[1].elements.push_back(m.groups[0].elements[3]);
    m.groups[0].elements.pop_back();
    m.groups[0].elements.pop_back();

    return m;
}

static Mesh buildTwoMaterialsMesh() 
{
    Mesh m;

    m.grid = {
        std::vector<double>{29.6508333333389, 29.9, 30.1, 30.3498270483333},
        std::vector<double>{-0.25, -6.21031004399696E-16, 0.249999999999999},
        std::vector<double>{  -0.25,  0, 0.249827048333335}
    };

    m.coordinates = {
        Coordinate({ +3.01000000e+01, +7.50000000e-01, 0.0}),
        Coordinate({ +3.00000000e+01, +7.50000000e-01, 0.0}),
        Coordinate({ +2.99000000e+01, +7.50000000e-01, 0.0}),
        Coordinate({ +3.01000000e+01, -7.50000000e-01, 0.0}),
        Coordinate({ +3.00000000e+01, -7.50000000e-01, 0.0}),
        Coordinate({ +2.99000000e+01, -7.50000000e-01, 0.0}),
        Coordinate({ +2.96500000e+01, +5.55111512e-16, 0.0})
    };

    m.groups = { {}, {} };
    m.groups[0].elements = {
        Element({4, 1, 6}, Element::Type::Surface)
    };
    m.groups[1].elements = {
        Element({3, 0, 5}, Element::Type::Surface),
        Element({0, 2, 5}, Element::Type::Surface)
    };

    return m;
}

static Mesh buildTwoTrianglesFromBowtieCoarseMesh() {
    const double boxMin = - 61.0e-3;
    const double boxMax =   64.0e-3;
    const double stepSize = 5.0e-3;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid = utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ -5.10000000e-2, 1.87712451e-2, 1.00000000e-3 }),
        Coordinate({ -5.10000000e-2, 1.81509036e-2, 0.00000000e-3 }),
        Coordinate({ -5.10000000e-2, 1.21755990e-2, 1.00000000e-3 }),
        Coordinate({ -5.10000000e-2, 1.14361869e-2, 0.00000000e-3 }),
    };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(2, tri)}) };
    groups[0].elements[0].vertices = { 0, 1, 2 };
    groups[0].elements[1].vertices = { 2, 1, 3 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildBowtieSubset1Mesh() 
{
    const double stepSize = 5.0e-3;
    Grid grid;
    grid[0] = utils::GridTools::linspace(-0.061, 0.064, 26);
    grid[1] = utils::GridTools::linspace(-0.059, 0.061, 25);
    grid[2] = utils::GridTools::linspace(-0.010, 0.015, 6);

    std::vector<Coordinate> coords = {
        Coordinate({ +4.25014971e-2, -2.058584271e-2, 0.00000000e-3 }),
        Coordinate({ +3.42365008e-2, -2.276349926e-2, 0.00000000e-3 }),
        Coordinate({ +2.60000000e-2, -2.500000000e-2, 0.00000000e-3 }),
        Coordinate({ +3.64730016e-2, -1.452699849e-2, 0.00000000e-3 }),
        Coordinate({ +2.82365008e-2, -1.676349929e-2, 0.00000000e-3 }), };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(3, tri)}) };
    groups[0].elements[0].vertices = { 1, 2, 4 };
    groups[0].elements[1].vertices = { 1, 4, 3 };
    groups[0].elements[2].vertices = { 1, 3, 0 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildCornerBowtieMesh(double stepSize)
{
    Grid grid;
    {
        std::size_t xyNum = (std::size_t)((120.0) / (double)stepSize) + 1;
        std::size_t zNum = (std::size_t)((5.0) / (double)stepSize) + 1;
        grid[0] = utils::GridTools::linspace(-60.0, 60.0, xyNum);
        grid[1] = grid[0];
        grid[2] = utils::GridTools::linspace(-1.0, 2.0, zNum);
    }

    std::vector<Coordinate> coords = {
        Coordinate({ +5.10000000e+01, -4.76385351e+01, +1.00000000e+00 }),
        Coordinate({ +5.10000000e+01, -4.76385351e+01, +0.00000000e+00 }),
        Coordinate({ +5.10000000e+01, -4.90000000e+01, +1.00000000e+00 }),
        Coordinate({ +5.00000000e+01, -4.90000000e+01, +1.00000000e+00 }),
        Coordinate({ +5.10000000e+01, -4.90000000e+01, +0.00000000e+00 }),
        Coordinate({ +5.00000000e+01, -4.90000000e+01, +0.00000000e+00 })
    };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(6, tri)}) };
    groups[0].elements[0].vertices = { 3, 5, 4 };
    groups[0].elements[1].vertices = { 4, 2, 3 };
    groups[0].elements[2].vertices = { 2, 4, 1 };
    groups[0].elements[3].vertices = { 0, 2, 1 };
    groups[0].elements[4].vertices = { 4, 5, 1 };
    groups[0].elements[5].vertices = { 2, 3, 0 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildCubeSurfaceMesh(double stepSize)
{
    const double boxMin = -1.0;
    const double boxMax = 2.0;
    assert(stepSize <= (boxMax - boxMin));
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid =
        utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    std::vector<Coordinate> coords = {
        Coordinate({ 0.00000000e+00, 1.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 0.00000000e+00, 0.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 0.00000000e+00, 1.00000000e+00, 0.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 1.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 0.00000000e+00, 1.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 1.00000000e+00, 0.00000000e+00 }),
        Coordinate({ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00 }),
        Coordinate({ 1.00000000e+00, 0.00000000e+00, 0.00000000e+00 })
    };

    Element tri;
    tri.type = Element::Type::Surface;
    std::vector<Group> groups = { Group({std::vector<Element>(12, tri)}) };
    groups[0].elements[0].vertices  = { 7  ,      6 ,       2 };
    groups[0].elements[1].vertices  = { 2  ,      5 ,       7 };
    groups[0].elements[2].vertices  = { 6  ,      7 ,       4 };
    groups[0].elements[3].vertices  = { 4  ,      1 ,       6 };
    groups[0].elements[4].vertices  = { 7  ,      5 ,       3 };
    groups[0].elements[5].vertices  = { 3  ,      4 ,       7 };
    groups[0].elements[6].vertices  = { 5  ,      2 ,       0 };
    groups[0].elements[7].vertices  = { 0  ,      3 ,       5 };
    groups[0].elements[8].vertices  = { 2  ,      6 ,       1 };
    groups[0].elements[9].vertices  = { 1  ,      0 ,       2 };
    groups[0].elements[10].vertices = { 1  ,      4 ,       3 };
    groups[0].elements[11].vertices = { 3  ,      0 ,       1 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildCubeVolumeMesh(double stepSize)
{
    const double boxMin = -1.0;
    const double boxMax = 2.0;
    const std::size_t gridLinesNum = (std::size_t)((boxMax - boxMin) / (double)stepSize) + 1;
    Grid grid =
        utils::GridTools::buildCartesianGrid(boxMin, boxMax, gridLinesNum);

    // Z  Y
    // | /
    // O - X

    //    5 --- 6
    //  / |   / |
    // 4 --- 7  |
    // |  |  |  |
    // |  1 -|- 2
    // | /   | /
    // 0 --- 3 

    std::vector<Coordinate> coords = {
        Coordinate({ 0.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 1.0, 0.0 }),
        Coordinate({ 1.0, 1.0, 0.0 }),
        Coordinate({ 1.0, 0.0, 0.0 }),
        Coordinate({ 0.0, 0.0, 1.0 }),
        Coordinate({ 0.0, 1.0, 1.0 }),
        Coordinate({ 1.0, 1.0, 1.0 }),
        Coordinate({ 1.0, 0.0, 1.0 })
    };

    Element tet;
    tet.type = Element::Type::Volume;
    std::vector<Group> groups = { Group({std::vector<Element>(5, tet)}) };
    groups[0].elements[0].vertices = { 0, 1, 3, 4 };
    groups[0].elements[1].vertices = { 3, 1, 2, 6 };
    groups[0].elements[2].vertices = { 3, 6, 4, 7 };
    groups[0].elements[3].vertices = { 4, 1, 5, 6 };
    groups[0].elements[4].vertices = { 6, 4, 3, 1 };

    return Mesh{ grid, coords, groups };
}

static Mesh buildSlabSurfaceMesh(double stepSize, double height)
{
    auto m{ buildCubeSurfaceMesh(stepSize) };
    for (auto& c : m.coordinates) {
        if (c(2) == 1.0) {
            c(2) = height;
        }
    }
    return m;
}

static Mesh buildCylinderPatchMesh()
{
    Mesh m;
    {
        std::size_t num = (std::size_t)((2.0 / 0.1) + 1);
        m.grid[0] = utils::GridTools::linspace(-1.0, 1.0, num);
        m.grid[1] = m.grid[0];
        m.grid[2] = utils::GridTools::linspace(-0.5, 1.5, num);
    }
    m.coordinates = {
        Coordinate({-4.80977621e-01, -1.36603545e-01, +2.54320758e-01}),
        Coordinate({-4.98211287e-01, -4.22553406e-02, +2.38152804e-01}),
        Coordinate({-4.97619843e-01, +4.87287602e-02, +2.47641174e-01}),
        Coordinate({-4.65985739e-01, -1.81265803e-01, +3.42455011e-01}),
        Coordinate({-4.90660449e-01, -9.61889986e-02, +3.19987579e-01}),
        Coordinate({-4.92206985e-01, +8.79334053e-02, +3.34696904e-01}),
        Coordinate({-4.99986632e-01, -3.65619741e-03, +3.28654490e-01}),
        Coordinate({-4.85717121e-01, -1.18654448e-01, +4.10616350e-01}),
        Coordinate({-4.96994803e-01, -5.47372419e-02, +4.11790266e-01}),
        Coordinate({-4.98653791e-01, +3.66660138e-02, +4.18162032e-01})
    };
    m.groups = { Group() };
    m.groups[0].elements = {
        Element({ 2, 5, 6 }, Element::Type::Surface),
        Element({ 6, 5, 9 }, Element::Type::Surface),
        Element({ 2, 6, 1 }, Element::Type::Surface),
        Element({ 1, 6, 4 }, Element::Type::Surface),
        Element({ 3, 0, 4 }, Element::Type::Surface),
        Element({ 4, 0, 1 }, Element::Type::Surface),
        Element({ 3, 4, 7 }, Element::Type::Surface),
        Element({ 7, 4, 8 }, Element::Type::Surface),
        Element({ 4, 6, 8 }, Element::Type::Surface)
    };
    return m;

}

static Mesh buildTwoCubesWithOffsetMesh(double stepSize)
{
    auto m{ buildCubeSurfaceMesh(1.0) };
    m.groups.resize(2);
    
    auto c2{ buildCubeSurfaceMesh(1.0) };
    std::transform(
        c2.coordinates.begin(), c2.coordinates.end(), 
        c2.coordinates.begin(),
        [&](const auto& v) { return v + VecD({ 0.25, 0.00, 0.00 }); }
    );
    c2.groups.resize(2);
    std::swap(c2.groups[0], c2.groups[1]);
    
    utils::meshTools::mergeMesh(m, c2);
    
    return m;
}

static Mesh buildProblematicTriMesh()
{
    Mesh m;
    m.grid = {
    std::vector<double>({
        -60.02108773095,
        -59.0275582070775,
        -58.034028683205,
        -57.0404991593326,
        -56.0469696354601,
        -55.0534401115876,
        -54.0599105877151,
        -53.0663810638426,
        -52.0728515399702,
        -51.0793220160977,
        -50.0857924922252,
        -49.1198666659361,
        -48.153940839647,
        -47.188015013358,
        -46.2220891870689,
        -45.2561633607798,
        -44.2902375344907,
        -43.3243117082017,
        -42.3583858819126,
        -41.3924600556235,
        -40.4265342293345,
        -39.4606084030454,
        -38.4946825767563,
        -37.5287567504673,
        -36.5628309241782,
        -35.5969050978891,
        -34.6309792716001,
        -33.665053445311,
        -32.6991276190219,
        -31.7332017927328,
        -30.7672759664438,
        -29.8013501401547,
        -28.8354243138657,
        -27.8694984875766,
        -26.9035726612875,
        -25.9376468349985,
        -24.9717210087094,
        -24.0057951824203,
        -23.0398693561313,
        -22.0739435298422,
        -21.1080177035531
    }),
    std::vector<double>({
        -9,
        -8,
        -7,
        -6,
        -5,
        -4,
        -3,
        -2,
        -1,
        -3.46944695195361E-15,
        0.999999999999997,
        2,
        3
    }),
    std::vector<double>({
        5.98379674014216,
        6.4667596532867,
        7.51937342222703,
        8.57198719116736
    })
    };
    m.coordinates = {
        Coordinate({ -2.84407720e+01, -9.00000000e+00, +7.93126475e+00 }),
        Coordinate({ -2.72927428e+01, -9.00000000e+00, +7.62365125e+00 }),
        Coordinate({ -2.11727225e+01, +2.64620374e+00, +5.98379674e+00 })
    };

    m.groups = { Group() };
    m.groups[0].elements = { Element({2, 1, 0}) };

    return m;
}

static Mesh buildProblematicTriMesh2() 
{
    Mesh m;
    m.grid = {
        std::vector<double>({
      -60.02108773095,
      -59.0275582070775,
      -58.034028683205,
      -57.0404991593326,
      -56.0469696354601,
      -55.0534401115876,
      -54.0599105877151,
      -53.0663810638426,
      -52.0728515399702,
      -51.0793220160977,
      -50.0857924922252,
      -49.1198666659361,
      -48.153940839647,
      -47.188015013358,
      -46.2220891870689,
      -45.2561633607798,
      -44.2902375344907,
      -43.3243117082017,
      -42.3583858819126,
      -41.3924600556235,
      -40.4265342293345,
      -39.4606084030454,
      -38.4946825767563,
      -37.5287567504673,
      -36.5628309241782,
      -35.5969050978891,
      -34.6309792716001,
      -33.665053445311,
      -32.6991276190219,
      -31.7332017927328,
      -30.7672759664438,
      -29.8013501401547,
      -28.8354243138657,
      -27.8694984875766,
      -26.9035726612875,
      -25.9376468349985,
      -24.9717210087094,
      -24.0057951824203,
      -23.0398693561313,
      -22.0739435298422,
      -21.1080177035531,
      -20.1304450202344,
      -19.1528723369158,
      -18.1752996535971,
      -17.1977269702784,
      -16.2201542869597,
      -15.242581603641,
      -14.2650089203223,
      -13.2874362370037,
      -12.309863553685,
      -11.3322908703663,
      -10.3663650440772,
      -9.40043921778816,
      -8.4345133914991,
      -7.46858756521003,
      -6.50266173892096,
      -5.5483827696615,
      -4.59410380040205,
      -3.63982483114259,
      -2.68554586188314,
      -1.73126689262368,
      -0.776987923364228,
      0.177291045895227,
      1.13157001515468,
      2.08584898441414,
      3.04012795367359,
      4.00605377996266,
      4.97197960625173,
      5.93790543254079,
      6.90383125882986,
      7.86975708511893,
      8.835682911408,
      9.80160873769707,
      10.7675345639861,
      11.7334603902752,
      12.6993862165643,
      13.6653120428533,
      14.6312378691424,
      15.5971636954315,
      16.5630895217205,
      17.5290153480096,
      18.4949411742987,
      19.4608670005877,
      20.4267928268768,
      21.3927186531659,
      22.358644479455,
      23.324570305744,
      24.2904961320331,
      25.2564219583222,
      26.2223477846112,
      27.1882736109003,
      28.1541994371894,
      29.1201252634784,
      30.0860510897675,
      31.0519769160566,
      32.0179027423456,
      33.0114322662181,
      34.0049617900905,
      34.998491313963,
      35.9920208378354,
      36.9855503617078,
      37.9790798855803,
      38.9726094094527,
      39.9661389333251,
      40.9596684571976,
      41.95319798107
        }),
        std::vector<double>({
      -39,
      -38,
      -37,
      -36,
      -35,
      -34,
      -33,
      -32,
      -31,
      -30,
      -29,
      -28,
      -27,
      -26,
      -25,
      -24,
      -23,
      -22,
      -21,
      -20,
      -19,
      -18,
      -17,
      -16,
      -15,
      -14,
      -13,
      -12,
      -11,
      -10,
      -9,
      -8,
      -7,
      -6,
      -5,
      -4,
      -3,
      -2,
      -1,
      -3.46944695195361E-15,
      0.999999999999997,
      2,
      3,
      4,
      5,
      6,
      7,
      8,
      9,
      10,
      11,
      12,
      13,
      14,
      15,
      16,
      17,
      18,
      19,
      20,
      21,
      22,
      23,
      24,
      25,
      26,
      27,
      28,
      29,
      30,
      31,
      32,
      33,
      34,
      35,
      36,
      37,
      38,
      39,
      40,
      41,
      42,
      43,
      44,
      45,
      46,
      47,
      48,
      49,
      50,
      51
        }),
        std::vector<double>({
      -18.2512507404964,
      -17.2512507404964,
      -16.2512507404964,
      -15.2512507404964,
      -14.2512507404964,
      -13.2512507404964,
      -12.2512507404964,
      -11.2512507404964,
      -10.2512507404964,
      -9.25125074049644,
      -8.25125074049645,
      -7.76828782735191,
      -6.71567405841158,
      -5.66306028947124,
      -5.18009737632671,
      -4.12748360738637,
      -3.07486983844604,
      -2.59190692530151,
      -1.53929315636117,
      -0.48667938742084,
      -3.71647427630561E-03,
      0.904008420720668,
      1.81173331571764,
      2.77765914200671,
      3.10582854123024,
      4.07175436751931,
      5.02777555383074,
      5.98379674014216,
      6.4667596532867,
      7.51937342222703,
      8.57198719116736,
      9.0549501043119,
      10.1075638732522,
      11.1601776421926,
      11.6431405553371,
      12.6957543242774,
      13.7483680932178,
      14.2313310063623,
      15.2313310063623,
      16.2313310063622,
      17.2313310063622,
      18.2313310063622,
      19.2313310063621,
      20.2313310063621,
      21.2313310063621,
      22.2313310063621,
      23.231331006362,
      24.231331006362
        })
    };
    m.coordinates = {
        Coordinate({  -3.17310478e+01, +2.02121840e+01, +8.81289149e+00 }),
        Coordinate({  -5.01504973e+01, +2.53737056e+01, +1.37483681e+01 }),
        Coordinate({  -3.98179903e+01, +3.10000000e+01, +1.09797812e+01 })
    };

    m.groups = { Group() };
    m.groups[0].elements = { Element({1, 0, 2}) };

    return m;
}

}
}