cmake_minimum_required(VERSION 3.20)

include_directories(".")

add_subdirectory(utils)

if (TESSELLATOR_USE_CGAL)
    add_subdirectory(cgal)
endif()

add_subdirectory(core)
add_subdirectory(drivers)