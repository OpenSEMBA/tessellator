cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator-utils")

add_library(tessellator-utils
    "Cleaner.cpp"
    "ConvexHull.cpp"
    "CoordGraph.cpp"
    "ElemGraph.cpp"
    "Geometry.cpp"
    "GridTools.cpp"
    "MeshTools.cpp"
    "Tools.cpp"
)

find_package(Boost REQUIRED graph)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(tessellator-utils Boost::graph)
