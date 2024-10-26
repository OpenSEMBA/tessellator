message(STATUS "Creating build system for tessellator-cgal")

add_library(tessellator-cgal
    "ConvexHull.cpp"
    "Delaunator.cpp"
    "HPolygonSet.cpp"
    "Manifolder.cpp"
    "ConvexHull.cpp"
    "PolyhedronTools.cpp"
    "Repairer.cpp"
    "Tools.cpp"
    "filler/Filler.cpp"
    "filler/FillerTools.cpp"
    "filler/SegmentsArray.cpp"
    "filler/Slice.cpp"
)

find_package(CGAL CONFIG REQUIRED)
target_link_libraries(tessellator-cgal CGAL::CGAL)
