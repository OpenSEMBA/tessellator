cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

set(CMAKE_CXX_STANDARD 17)

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
