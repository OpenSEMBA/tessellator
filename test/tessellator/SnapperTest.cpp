#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "Snapper.h"
#include "Slicer.h"
#include "utils/Tools.h"
#include "utils/Geometry.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace utils;
using namespace meshFixtures;

class SnapperTest : public ::testing::Test {
protected:
};
//
//TEST_F(SnapperTest, selfOverlapping_stepSize1)
//{
//	ASSERT_NO_THROW(Snapper{ Slicer{ buildSelfOverlappingMesh(1.0) }.getMesh()	});
//}
//
//TEST_F(SnapperTest, selfOverlapping_stepSize0c25)
//{
//	ASSERT_NO_THROW(Snapper{ Slicer{ buildSelfOverlappingMesh(0.25) }.getMesh() });
//}
//
//TEST_F(SnapperTest, selfOverlapping_partially_stepSize1)
//{
//	auto m{ buildSelfOverlappingMesh(1.0) };
//	m.coordinates[3] = Coordinate({ 0.9, 0.9, 0.0 });
//
//	ASSERT_NO_THROW(Snapper{ Slicer{m}.getMesh() });
//}