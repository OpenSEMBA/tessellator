cmake_minimum_required(VERSION 3.20)

message(STATUS "Creating build system for tessellator-drivers")
include_directories("../core/")

add_library(tessellator-drivers
    "DriverBase.cpp"
    "StructuredDriver.cpp"
)
target_link_libraries(tessellator-drivers tessellator-core tessellator-utils Boost::graph)

if (TESSELLATOR_USE_CGAL)
    add_library(tessellator "Driver.cpp")
    target_link_libraries(tessellator-drivers tessellator-cgal)
endif()

if(TESSELLATOR_EXECUTION_POLICIES)
    add_definitions(-DTESSELLATOR_EXECUTION_POLICIES)
    find_package(TBB CONFIG REQUIRED)
    target_link_libraries(tessellator-drivers TBB::tbb)
endif()