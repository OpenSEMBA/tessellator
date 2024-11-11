#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "cgal/Manifolder.h"
#include "cgal/Repairer.h"
#include "cgal/PolyhedronTools.h"

#include "utils/MeshTools.h"

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

namespace meshlib::cgal {

using namespace polyhedronTools;
using namespace meshFixtures;
using namespace utils::meshTools;

class RepairTest : public ::testing::Test {
protected:
	static bool sameCoordinatesNotNecessarilyOrdered(const Coordinates& as, const Coordinates& bs)
	{
		return std::set<Coordinate>(as.begin(), as.end()) ==
			std::set<Coordinate>(bs.begin(), bs.end());
	}

	static Mesh readMeshFromFile(const std::string& filename)
	{
		Polyhedron p;
		if (!PMP::IO::read_polygon_mesh("./testData/" + filename, p))
		{
			throw std::logic_error("Input file does not exist");
		}
		return buildMeshFromPolyhedron(p);
	}

	static void fillHolesInSTL(const std::string& filename)
	{
		CGAL::IO::write_STL(
			readNameWithoutExtension(filename) + "_out.stl",
			buildPolyhedronFromMesh(
				repair( readMeshFromFile(filename) ))
		);
	}

	static std::string readNameWithoutExtension(const std::string& filename) {
		return filename.substr(0, filename.find_last_of("."));
	}

	static void addOffset(Coordinates& cs, Coordinate offset)
	{
		std::transform(
			cs.begin(), cs.end(),
			cs.begin(),
			[&](auto& c) { return c + offset; }
		);
	}

	static bool noChangesWhenRepair(const std::string& fn)
	{
		auto m{ readMeshFromFile(fn) };
		Manifolder mf{ m };
		Manifolder mfr{ repair( m ) };
		return mf.getClosedSurfacesMesh().countElems() 
			== mfr.getClosedSurfacesMesh().countElems() 
			&& mf.getOpenSurfacesMesh().countElems() 
			== mfr.getOpenSurfacesMesh().countElems();
	}
};

TEST_F(RepairTest, fill_sphere_one_gap_from_stl)
{
	ASSERT_NO_THROW(fillHolesInSTL("open_sphere.stl"));
}

TEST_F(RepairTest, fill_sphere_many_gaps_from_stl)
{
	ASSERT_NO_THROW(fillHolesInSTL("open_sphere_2.stl"));
}

TEST_F(RepairTest, fill_half_sphere_from_stl)
{
	ASSERT_NO_THROW(fillHolesInSTL("open_sphere_half.stl"));
}

TEST_F(RepairTest, fill_cube)
{
	auto m{ buildCubeSurfaceMesh(1.0) };
	auto r{ repair(m) };

	EXPECT_TRUE(sameCoordinatesNotNecessarilyOrdered(m.coordinates, r.coordinates));
}

TEST_F(RepairTest, fill_tet)
{
	auto m{ buildTetMesh(1.0) };
	auto r{ repair(m) };

	EXPECT_TRUE(sameCoordinatesNotNecessarilyOrdered(m.coordinates, r.coordinates));
	
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(1, r.groups[0].elements.size());
	EXPECT_EQ(1, countMeshElementsIf(r, isTetrahedron));
}

TEST_F(RepairTest, fill_tet_surface)
{
	auto m{ Manifolder{buildTetMesh(1.0)}.getClosedSurfacesMesh() };
	auto r{ repair(m) };

	EXPECT_TRUE(sameCoordinatesNotNecessarilyOrdered(m.coordinates, r.coordinates));
	
	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(4, r.groups[0].elements.size());
}

TEST_F(RepairTest, fill_different_connected_components)
{
	Mesh m;
	{
		m = buildTetMesh(1.0);
		m.grid[0] = { 0.0, 1.0, 2.0, 3.0 };
		addOffset(m.coordinates, Coordinate({2.0, 0.0, 0.0}));
		auto m2{ m };
		utils::meshTools::mergeMesh(m, m2);
	    m = Manifolder{m}.getClosedSurfacesMesh();
	}

	auto r{ repair(m) };

	EXPECT_TRUE(sameCoordinatesNotNecessarilyOrdered(m.coordinates, r.coordinates));

	ASSERT_EQ(1, r.groups.size());
	EXPECT_EQ(8, r.groups[0].elements.size());
}

TEST_F(RepairTest, throw_if_self_intersections)
{
	ASSERT_ANY_THROW( 
		repair( readMeshFromFile("Substrate_self_intersections.stl") ) 
	);
}

}