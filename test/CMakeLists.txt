message(STATUS "Creating build system for tessellator-tests")

find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


include_directories(
	${PROJECT_SOURCE_DIR}/src/
	${PROJECT_SOURCE_DIR}/src/app/
	${PROJECT_SOURCE_DIR}/src/core/
	${PROJECT_SOURCE_DIR}/src/meshers/
	${PROJECT_SOURCE_DIR}/src/utils/
	${PROJECT_SOURCE_DIR}/src/types/
	${PROJECT_SOURCE_DIR}/test/
)

add_executable(tessellator_tests 
	"app/launcherTest.cpp"
	"app/vtkIOTest.cpp"
	"core/CollapserTest.cpp"
	"core/SlicerTest.cpp"
	"core/SnapperTest.cpp"
	"core/SmootherTest.cpp"
	"core/SmootherToolsTest.cpp"
    "core/StaircaserTest.cpp"
	"types/MeshTest.cpp"
	"utils/ConvexHullTest.cpp"
	"utils/CoordGraphTest.cpp"
	"utils/ElemGraphTest.cpp"
	"utils/GeometryTest.cpp"
	"utils/GridToolsTest.cpp"
	"utils/MeshToolsTest.cpp"
	"utils/RedundancyCleanerTest.cpp"
    "meshers/StaircaseMesherTest.cpp"
	"meshers/OffgridMesherTest.cpp"
	"meshers/ConformalMesherTest.cpp"
)

target_link_libraries(tessellator_tests	
	tessellator-meshers
	tessellator-app
    GTest::gtest 
	GTest::gtest_main
)

if (TESSELLATOR_ENABLE_CGAL)
	include_directories(
		${PROJECT_SOURCE_DIR}/src/cgal/
	)

	target_sources(tessellator_tests PRIVATE
		"cgal/ArrangmentTest.cpp"
		"cgal/DelaunatorTest.cpp"
		"cgal/HPolygonSetTest.cpp"
		"cgal/ManifolderTest.cpp"
		"cgal/PolyhedronToolsTest.cpp"
		"cgal/RepairerTest.cpp"
		"cgal/filler/FillerTest.cpp"
		"cgal/filler/FillerToolsTest.cpp"
	)

	target_link_libraries(tessellator_tests tessellator-cgal)
endif()

add_test(tessellator ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tessellator_tests)