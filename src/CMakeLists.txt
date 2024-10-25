cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator")

add_library(tessellator-utils
    "utils/Cleaner.cpp"
    "utils/CoordGraph.cpp"
    "utils/ElemGraph.cpp"
    "utils/Geometry.cpp"
    "utils/GridTools.cpp"
    "utils/MeshTools.cpp"
    "utils/Tools.cpp"
)

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
        "filler/Filler.cpp"
        "filler/FillerTools.cpp"
        "filler/SegmentsArray.cpp"
        "filler/Slice.cpp"
    )

    find_package(CGAL CONFIG REQUIRED)
    target_link_libraries(tessellator-cgal tessellator-utils CGAL::CGAL)
endif()

add_library(tessellator
    "tessellator/Collapser.cpp"
    "tessellator/Driver.cpp"
    "tessellator/Slicer.cpp"
    "tessellator/Smoother.cpp"
    "tessellator/SmootherTools.cpp"
    "tessellator/Snapper.cpp"
)


include_directories(".")

find_package(Boost REQUIRED graph)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(tessellator tessellator-utils Boost::graph)

if (TESSELLATOR_USE_CGAL)
    target_link_libraries(tessellator tessellator-cgal)
endif()

if(TESSELLATOR_EXECUTION_POLICIES)
    add_definitions(-DTESSELLATOR_EXECUTION_POLICIES)
    find_package(TBB CONFIG REQUIRED)
    target_link_libraries(tessellator TBB::tbb)
endif()