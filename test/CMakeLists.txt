cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator_tests")

find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(
	${PROJECT_SOURCE_DIR}/src/
	${PROJECT_SOURCE_DIR}/src/tessellator/
	${PROJECT_SOURCE_DIR}/src/cgal/
	${PROJECT_SOURCE_DIR}/src/utils/
	${PROJECT_SOURCE_DIR}/src/types/
	${PROJECT_SOURCE_DIR}/test/
)

add_executable(tessellator_tests 
	"cgal/ArrangmentTest.cpp"
	"cgal/ConvexHullTest.cpp"
	"cgal/DelaunatorTest.cpp"
	"cgal/HPolygonSetTest.cpp"
	"cgal/LSFPlaneTest.cpp"
	"cgal/ManifolderTest.cpp"
	"cgal/PolyhedronToolsTest.cpp"
	"cgal/RepairerTest.cpp"
	"tessellator/filler/FillerTest.cpp"
	"tessellator/filler/FillerToolsTest.cpp"
	"tessellator/CollapserTest.cpp"
	"tessellator/DriverTest.cpp"
	"tessellator/SlicerTest.cpp"
	"tessellator/SmootherTest.cpp"
	"tessellator/SmootherToolsTest.cpp"
	"tessellator/SnapperTest.cpp"
	"types/MeshTest.cpp"
	"utils/CleanerTest.cpp"
	"utils/CoordGraphTest.cpp"
	"utils/ElemGraphTest.cpp"
	"utils/GeometryTest.cpp"
	"utils/GridToolsTest.cpp"
	"utils/MeshToolsTest.cpp"
)

target_link_libraries(tessellator_tests	
	tessellator
    GTest::gtest 
	GTest::gtest_main
)

add_test(tessellator ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tessellator_tests)