cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator")

include_directories(".")

add_library(tessellator-utils
    "utils/Cleaner.cpp"
    "utils/CoordGraph.cpp"
    "utils/ElemGraph.cpp"
    "utils/Geometry.cpp"
    "utils/GridTools.cpp"
    "utils/MeshTools.cpp"
    "utils/Tools.cpp"
)

find_package(Boost REQUIRED graph)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(tessellator-utils Boost::graph)

if (TESSELLATOR_USE_CGAL)
    add_library(tessellator-cgal
        "cgal/ConvexHull.cpp"
        "cgal/Delaunator.cpp"
        "cgal/HPolygonSet.cpp"
        "cgal/Manifolder.cpp"
        "cgal/ConvexHull.cpp"
        "cgal/PolyhedronTools.cpp"
        "cgal/Repairer.cpp"
        "cgal/Tools.cpp"
        "cgal/filler/Filler.cpp"
        "cgal/filler/FillerTools.cpp"
        "cgal/filler/SegmentsArray.cpp"
        "cgal/filler/Slice.cpp"
    )

    find_package(CGAL CONFIG REQUIRED)
    target_link_libraries(tessellator-cgal CGAL::CGAL)
endif()

add_library(tessellator-core
    "core/Collapser.cpp"
    "core/Slicer.cpp"
    "core/Smoother.cpp"
    "core/SmootherTools.cpp"
    "core/Snapper.cpp"
    "core/Structurer.cpp"
)

add_subdirectory(drivers)