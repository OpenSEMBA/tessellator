cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator_tests")

find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(
	${PROJECT_SOURCE_DIR}/src/
	${PROJECT_SOURCE_DIR}/src/core/
	${PROJECT_SOURCE_DIR}/src/drivers/
	${PROJECT_SOURCE_DIR}/src/utils/
	${PROJECT_SOURCE_DIR}/src/types/
	${PROJECT_SOURCE_DIR}/test/
)

add_executable(tessellator_tests 
	"core/CollapserTest.cpp"
	"core/SlicerTest.cpp"
	"core/SnapperTest.cpp"
    "core/StructurerTest.cpp"
	"types/MeshTest.cpp"
	"utils/CleanerTest.cpp"
	"utils/ConvexHullTest.cpp"
	"utils/CoordGraphTest.cpp"
	"utils/ElemGraphTest.cpp"
	"utils/GeometryTest.cpp"
	"utils/GridToolsTest.cpp"
	"utils/MeshToolsTest.cpp"
    "drivers/StructuredDriverTest.cpp"
)

target_link_libraries(tessellator_tests	
	tessellator-drivers
    GTest::gtest 
	GTest::gtest_main
)

if (TESSELLATOR_USE_CGAL)
	include_directories(
		${PROJECT_SOURCE_DIR}/src/cgal/
	)

	target_sources(tessellator_tests PRIVATE
		"cgal/ArrangmentTest.cpp"
		"cgal/DelaunatorTest.cpp"
		"cgal/HPolygonSetTest.cpp"
		"cgal/LSFPlaneTest.cpp"
		"cgal/ManifolderTest.cpp"
		"cgal/PolyhedronToolsTest.cpp"
		"cgal/RepairerTest.cpp"
		"cgal/filler/FillerTest.cpp"
		"cgal/filler/FillerToolsTest.cpp"
		"core/SmootherTest.cpp"
		"core/SmootherToolsTest.cpp"
		"drivers/OffgridDriverTest.cpp"
	)	
endif()

add_test(tessellator ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tessellator_tests)