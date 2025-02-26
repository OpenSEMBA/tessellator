message(STATUS "Creating build system for tessellator-core")

add_library(tessellator-core
    "Collapser.cpp"
    "Slicer.cpp"
    "Snapper.cpp"
    "Structurer.cpp"
    "Smoother.cpp"
    "SmootherTools.cpp"
)

target_link_libraries(tessellator-core tessellator-utils)