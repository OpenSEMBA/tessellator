message(STATUS "Creating build system for tessellator-core")

add_library(tessellator-core
    "Collapser.cpp"
    "Slicer.cpp"
    "Snapper.cpp"
    "Structurer.cpp"
)

if (TESSELLATOR_USE_CGAL)
    target_sources(tessellator-core PRIVATE
        "Smoother.cpp"
        "SmootherTools.cpp"
    )
    target_link_libraries(tessellator-core tessellator-cgal)
endif()

target_link_libraries(tessellator-core tessellator-utils)