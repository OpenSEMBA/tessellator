#include "gtest/gtest.h"

#include "MeshFixtures.h"

#include "cgal/Manifolder.h"
#include "Slicer.h"
#include "Collapser.h"
#include "Smoother.h"

#include "utils/Geometry.h"
#include "MeshTools.h"


using namespace meshlib;
using namespace tessellator;
using namespace cgal;
using namespace meshFixtures;

class ManifolderTest : public ::testing::Test {
public:
	static IdSet usedCoordIds(const Mesh& m)
	{
		IdSet usedCoordIds;
		for (auto const& g : m.groups) {
			for (auto const& e : g.elements) {
				usedCoordIds.insert(e.vertices.begin(), e.vertices.end());
			}
		}
		return usedCoordIds;
	}

	static auto usedDifferentCoords(const Mesh& m)
	{
		std::set<Coordinate> diffCoords;
		for (auto const& id : usedCoordIds(m)) {
			diffCoords.insert(m.coordinates[id]);
		}
		return diffCoords;
	}
	
	static bool sameCoordinatesNotNecessarilyOrdered(const Coordinates& as, const Coordinates& bs)
	{
		return std::set<Coordinate>(as.begin(), as.end()) ==
			std::set<Coordinate>(bs.begin(), bs.end());
	}
};

TEST_F(ManifolderTest, duplicateCoordinatesWhenEdgeShared)
{
	Mesh m = buildTetsSharingEdgeMesh();
	Mesh r = Manifolder(m).getClosedSurfacesMesh();
	
	EXPECT_EQ(6, m.coordinates.size());
	EXPECT_EQ(8, r.coordinates.size());

	EXPECT_EQ(r.coordinates.size(), usedCoordIds(r).size());
	EXPECT_EQ(m.coordinates.size(), usedDifferentCoords(r).size());
}

TEST_F(ManifolderTest, open_surface)
{
	Mesh m = buildPlane45Mesh(0.1);
	
	Manifolder oOH(m);
	Mesh rClosed = oOH.getClosedSurfacesMesh();
	Mesh rOpen = oOH.getOpenSurfacesMesh();
	
	EXPECT_EQ(0,              rClosed.countElems());
	EXPECT_EQ(m.countElems(), rOpen.countElems());
	EXPECT_EQ(rClosed.grid, rOpen.grid);

	EXPECT_EQ(rOpen.countElems(), oOH.getSurfacesMesh().countElems());
	EXPECT_EQ(rOpen.coordinates, oOH.getSurfacesMesh().coordinates);
	EXPECT_EQ(rOpen.grid, oOH.getSurfacesMesh().grid);
} 

TEST_F(ManifolderTest, volume_and_surface)
{
	Mesh m = buildTetAndTriMesh(1.0);

	Manifolder mani(m);
	
	ASSERT_EQ(4, mani.getClosedSurfacesMesh().countElems());
	EXPECT_EQ(4, usedDifferentCoords(mani.getClosedSurfacesMesh()).size());

	Mesh r = mani.getOpenSurfacesMesh();
	ASSERT_EQ(1, r.groups.size());
	ASSERT_EQ(1, r.countElems());
}

TEST_F(ManifolderTest, closed_surface)
{
	Mesh m = buildCubeSurfaceMesh(1.0);

	Mesh r;
	ASSERT_NO_THROW(r = Manifolder(m).getClosedSurfacesMesh());

	EXPECT_EQ(m.countElems(), r.countElems());
	EXPECT_EQ(m.coordinates.size(), r.coordinates.size());
}

TEST_F(ManifolderTest, open_surfaces_preserve_orientation)
{
	Mesh m = buildPlane45TwoMaterialsMesh(1.0);

	Mesh r = Manifolder(m).getOpenSurfacesMesh();

	EXPECT_TRUE(sameCoordinatesNotNecessarilyOrdered(m.coordinates, r.coordinates));
	EXPECT_EQ(m.grid, r.grid);
	ASSERT_EQ(2, r.groups.size());
	ASSERT_EQ(m.groups.size(), r.groups.size());

	VecD n = utils::Geometry::normal(
		utils::Geometry::asTriV(m.groups[0].elements[0], m.coordinates)
	);
	for (auto const& g : r.groups) {
		for (auto const& e : g.elements) {
			TriV tri = utils::Geometry::asTriV(e, r.coordinates);
			VecD n2 = utils::Geometry::normal(tri);
			EXPECT_NEAR(0.0, n.angleDeg(n2), 1e-3);
		}
	}
}

TEST_F(ManifolderTest, one_tet) 
{
	Mesh r;
	{
		Mesh m;
		m.coordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({0.0, 0.0, 1.0})
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({0, 1, 2, 3}, Element::Type::Volume)
		};

		r = Manifolder(m).getClosedSurfacesMesh();
	}
	
	EXPECT_EQ(4, r.countElems());
	EXPECT_EQ(4, r.coordinates.size());
	EXPECT_EQ(4, r.countTriangles());
}

TEST_F(ManifolderTest, two_connected_tets) 
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({0.0, 0.0, 1.0}),
			Coordinate({1.0, 1.0, 1.0})
		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({ 0, 1, 2, 3 }, Element::Type::Volume),
			Element({ 4, 1, 2, 3 }, Element::Type::Volume)
		};
	}
	Mesh r = Manifolder(m).getClosedSurfacesMesh();

	EXPECT_EQ(6, r.countTriangles());
	EXPECT_EQ(5, r.coordinates.size());

	EXPECT_EQ(r.coordinates.size(), usedCoordIds(r).size());
	EXPECT_EQ(m.coordinates.size(), usedDifferentCoords(r).size());
}

TEST_F(ManifolderTest, two_disconnected_tets)
{
	Mesh m;
	{
		m.coordinates = {
			Coordinate({ 0.0,  0.0,  0.0}),
			Coordinate({ 1.0,  0.0,  0.0}),
			Coordinate({ 0.0,  1.0,  0.0}),
			Coordinate({ 0.0,  0.0,  1.0}),
			Coordinate({10.0, 10.0, 10.0}),
			Coordinate({11.0, 10.0, 10.0}),
			Coordinate({10.0, 11.0, 10.0}),
			Coordinate({10.0, 10.0, 11.0})

		};
		m.groups = { Group() };
		m.groups[0].elements = {
			Element({ 0, 1, 2, 3 }, Element::Type::Volume),
			Element({ 4, 5, 6, 7 }, Element::Type::Volume)
		};
	}
	Mesh r = Manifolder(m).getClosedSurfacesMesh();

	EXPECT_EQ(8, r.countTriangles());
	EXPECT_EQ(8, r.coordinates.size());

	EXPECT_EQ(r.coordinates.size(), usedCoordIds(r).size());
	EXPECT_EQ(m.coordinates.size(), usedDifferentCoords(r).size());
}

TEST_F(ManifolderTest, tets_with_inner_point) 
{
	Mesh r = Manifolder(buildTetMeshWithInnerPoint(1.0)).getClosedSurfacesMesh();
	
	EXPECT_EQ(4, r.countTriangles());
	EXPECT_EQ(0, r.countElems() - r.countTriangles());
	EXPECT_EQ(4, usedCoordIds(r).size());

	ASSERT_EQ(1, r.groups.size());
	ASSERT_EQ(4, r.groups[0].elements.size());
	EXPECT_EQ(CoordinateIds({ 1, 0, 2 }), r.groups[0].elements[0].vertices);
	EXPECT_EQ(CoordinateIds({ 3, 1, 2 }), r.groups[0].elements[1].vertices);
	EXPECT_EQ(CoordinateIds({ 1, 3, 0 }), r.groups[0].elements[2].vertices);
	EXPECT_EQ(CoordinateIds({ 3, 2, 0 }), r.groups[0].elements[3].vertices);

}

TEST_F(ManifolderTest, emptyMesh)
{
	Mesh m;
	ASSERT_NO_THROW(Manifolder manifolder(m));
}

TEST_F(ManifolderTest, cubeMesh)
{
	Mesh m = buildCubeVolumeMesh(1.0);
	Mesh r = Manifolder(m).getClosedSurfacesMesh();

	EXPECT_EQ(12, r.countTriangles());
	EXPECT_EQ(0, r.countElems() - r.countTriangles());

	EXPECT_EQ(0, Manifolder(m).getOpenSurfacesMesh().countElems());
}

TEST_F(ManifolderTest, selfIntersecting_surface)
{
	EXPECT_EQ(2, 
		Manifolder{ buildSelfOverlappingMesh(1.0) }.getSurfacesMesh().countElems());
}

TEST_F(ManifolderTest, smashed_tet)
{
	Mesh m;
	m.coordinates = {
		Coordinate({17.25, 17.00, 2.00}),
		Coordinate({18.00, 16.75, 2.00}),
		Coordinate({18.00, 17.00, 2.00}),
		Coordinate({19.00, 16.75, 2.00})
	};
	m.groups = { Group() };
	m.groups[0].elements = {
		Element({1, 0, 2}),
		Element({1, 2, 3}),
		Element({2, 1, 3}),
		Element({2, 0, 1})
	};
	
	Manifolder mf{ m };
	
	EXPECT_EQ(2, mf.getOpenSurfacesMesh().countTriangles());
	EXPECT_EQ(0, mf.getClosedSurfacesMesh().countTriangles());
}