#include "gtest/gtest.h"

#include "MeshFixtures.h"

#include "cgal/PolyhedronTools.h"
#include "MeshTools.h"

namespace meshlib::cgal {

using namespace meshFixtures;

using namespace polyhedronTools;

class PolyhedronToolsTest: public ::testing::Test {
public:

};

TEST_F(PolyhedronToolsTest, buildPolyhedronFromMesh)
{
	auto p{ buildPolyhedronFromMesh(buildCubeSurfaceMesh(1.0)) };

	EXPECT_NE(0, p.size_of_vertices());
	EXPECT_NE(0, p.size_of_facets());
}


}