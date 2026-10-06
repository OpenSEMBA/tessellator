message(STATUS "Creating build system for tessellator-benchmarks")

if(NOT TESSELLATOR_ENABLE_APP)
    message(FATAL_ERROR "TESSELLATOR_ENABLE_BENCHMARKS requires TESSELLATOR_ENABLE_APP=ON (the sphere STL is loaded through vtkIO)")
endif()

add_executable(tessellator_benchmarks
    "StaircaseMesherBenchmark.cpp"
)

target_include_directories(tessellator_benchmarks PRIVATE
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/src/app/
    ${PROJECT_SOURCE_DIR}/src/meshers/
    ${PROJECT_SOURCE_DIR}/src/utils/
    ${PROJECT_SOURCE_DIR}/src/types/
    ${CMAKE_CURRENT_BINARY_DIR}/../src/app/
)

target_compile_definitions(tessellator_benchmarks PRIVATE
    TESSELLATOR_DATA_DIR="${PROJECT_SOURCE_DIR}/testData"
)

target_link_libraries(tessellator_benchmarks
    tessellator-meshers
    tessellator-app
)
