message(STATUS "Creating build system for tessellator-meshers")

add_library(tessellator-meshers
    "MesherBase.cpp"
    "StructuredMesher.cpp"
    "OffgridMesher.cpp"
)
target_link_libraries(tessellator-meshers tessellator-core tessellator-utils)

if(TESSELLATOR_EXECUTION_POLICIES)
    add_definitions(-DTESSELLATOR_EXECUTION_POLICIES)
    find_package(TBB CONFIG REQUIRED)
    target_link_libraries(tessellator-meshers TBB::tbb)
endif()