cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

set(CMAKE_CXX_STANDARD 17)

# Windows resource file configuration
set(PROJECT_COMPANY_NAME "UGR/Elemwave")
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_DESCRIPTION "Tessellator - Mesh generation library")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


option(TESSELLATOR_ENABLE_TESTS "Compile tests" ON)
option(TESSELLATOR_ENABLE_CGAL "Compile using CGAL library" ON)
option(TESSELLATOR_EXECUTION_POLICIES OFF)

if(TESSELLATOR_ENABLE_CGAL)
    list(APPEND VCPKG_MANIFEST_FEATURES "cgal")
endif()
					  
project(tessellator CXX)

add_subdirectory(src/)
					  
if(TESSELLATOR_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test/)
    add_test(tessellator ${CMAKE_BINARY_DIR}/tessellator_tests)
endif()
