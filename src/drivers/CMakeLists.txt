cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator-drivers")

add_library(tessellator-drivers
    "DriverBase.cpp"
    "StructuredDriver.cpp"
)
target_link_libraries(tessellator-drivers tessellator-core tessellator-utils)

if (TESSELLATOR_USE_CGAL)
    target_sources(tessellator-drivers PRIVATE
        "OffgridDriver.cpp"
    )
    target_link_libraries(tessellator-drivers tessellator-cgal)
endif()

if(TESSELLATOR_EXECUTION_POLICIES)
    add_definitions(-DTESSELLATOR_EXECUTION_POLICIES)
    find_package(TBB CONFIG REQUIRED)
    target_link_libraries(tessellator-drivers TBB::tbb)
endif()