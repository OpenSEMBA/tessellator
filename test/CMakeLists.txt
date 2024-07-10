cmake_minimum_required(VERSION 3.0)

message(STATUS "Creating build system for tessellator_tests")

find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

unset(SRCS)

add_sources_local(types/ SRCS)
add_sources_local(utils/ SRCS)
add_sources_local(tessellator/ SRCS)
add_sources_local(cgal/ SRCS)
add_sources_local(tessellator/filler/ SRCS)

include_directories(
	./
	../src/types
	../src/utils
	../src/tessellator)  
	
add_executable(tessellator_tests ${SRCS})


target_link_libraries(tessellator_tests	tessellator
                                    GTest::gtest GTest::gtest_main)

add_test(tessellator ${CMAKE_BINARY_DIR}/bin/tessellator_tests)