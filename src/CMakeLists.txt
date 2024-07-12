cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator")

add_library(tessellator
    "cgal/ConvexHull.cpp"
    "cgal/Delaunator.cpp"
    "cgal/HPolygonSet.cpp"
    "cgal/Manifolder.cpp"
    "cgal/ConvexHull.cpp"
    "cgal/PolyhedronTools.cpp"
    "cgal/Repairer.cpp"
    "cgal/Tools.cpp"
    "utils/Cleaner.cpp"
    "utils/CoordGraph.cpp"
    "utils/ElemGraph.cpp"
    "utils/Geometry.cpp"
    "utils/GridTools.cpp"
    "utils/MeshTools.cpp"
    "utils/Tools.cpp"
    "tessellator/Collapser.cpp"
    "tessellator/Driver.cpp"
    "tessellator/Slicer.cpp"
    "tessellator/Smoother.cpp"
    "tessellator/SmootherTools.cpp"
    "tessellator/Snapper.cpp"
    "tessellator/filler/Filler.cpp"
    "tessellator/filler/FillerTools.cpp"
    "tessellator/filler/SegmentsArray.cpp"
    "tessellator/filler/Slice.cpp"
)                       
include_directories(".")

find_package(Boost REQUIRED graph)
include_directories(${Boost_INCLUDE_DIRS})

find_package(CGAL CONFIG REQUIRED)
target_link_libraries(tessellator Boost::graph CGAL::CGAL)

if(TESSELLATOR_EXECUTION_POLICIES)
    add_definitions(-DTESSELLATOR_EXECUTION_POLICIES)
    find_package(TBB CONFIG REQUIRED)
    target_link_libraries(tessellator TBB::tbb)
endif()