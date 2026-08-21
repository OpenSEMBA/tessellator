cmake_minimum_required(VERSION 3.20)

# Toolchain detection: support both vcpkg (local dev) and system libraries (Docker)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# If CMAKE_TOOLCHAIN_FILE is set, verify it exists (avoid silent failures)
if(DEFINED CMAKE_TOOLCHAIN_FILE AND NOT EXISTS "${CMAKE_TOOLCHAIN_FILE}")
   message(WARNING "CMAKE_TOOLCHAIN_FILE specified but not found: ${CMAKE_TOOLCHAIN_FILE}")
   message(STATUS "Falling back to system libraries (Docker mode)")
   unset(CMAKE_TOOLCHAIN_FILE)
endif()

if(DEFINED CMAKE_TOOLCHAIN_FILE)
   message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
else()
   message(STATUS "Using system libraries (vcpkg toolchain not detected)")
endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)


option(TESSELLATOR_ENABLE_TESTS "Compile tests" ON)
option(TESSELLATOR_ENABLE_CGAL "Compile using CGAL library" OFF)
option(TESSELLATOR_EXECUTION_POLICIES OFF)
option(DOCKER_EXPORT_COMPILE_COMMANDS "Information for the 'clangd' linter extension" OFF)
option(TESSELLATOR_ENABLE_APP "Compile the tessellator launcher application" ON)

if(DOCKER_EXPORT_COMPILE_COMMANDS)
   set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()
if(TESSELLATOR_ENABLE_CGAL)
    list(APPEND VCPKG_MANIFEST_FEATURES "cgal")
    SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
    if (EIGEN3_INCLUDE_DIR)
      INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
      INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}/Eigen" )
    endif()
endif()
					  
project(tessellator VERSION 1.0.0 LANGUAGES CXX)

if(TESSELLATOR_ENABLE_APP)
   find_package(VTK REQUIRED COMPONENTS
      CommonCore
      IOGeometry
      IOLegacy
      FiltersCore
   )
   add_compile_definitions(APP_LOADED=1)
else()
   add_compile_definitions(APP_LOADED=0)
endif()

add_subdirectory(src/)
					  
if(TESSELLATOR_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test/)
endif()
