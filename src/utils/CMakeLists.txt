message(STATUS "Creating build system for tessellator-utils")

add_library(tessellator-utils
    "ConvexHull.cpp"
    "CoordGraph.cpp"
    "ElemGraph.cpp"
    "Geometry.cpp"
    "GridTools.cpp"
    "MeshTools.cpp"
    "RedundancyCleaner.cpp"
    "Tools.cpp"
)

find_package(Boost REQUIRED graph)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(tessellator-utils Boost::graph)
