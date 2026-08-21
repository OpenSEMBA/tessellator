message(STATUS "Creating build system for tessellator-app")

if(TESSELLATOR_ENABLE_APP)
    function(tessellator_escape_cpp_string input output)
        set(value "${input}")
        string(REPLACE "\\" "\\\\" value "${value}")
        string(REPLACE "\"" "\\\"" value "${value}")
        string(REPLACE "\n" "\\n" value "${value}")
        string(REPLACE "\r" "\\r" value "${value}")
        set(${output} "${value}" PARENT_SCOPE)
    endfunction()

    set(TESSELLATOR_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
    if(NOT TESSELLATOR_BUILD_TYPE)
        set(TESSELLATOR_BUILD_TYPE "multi-config")
    endif()
    tessellator_escape_cpp_string("${PROJECT_VERSION}" TESSELLATOR_PROJECT_VERSION_ESCAPED)
    tessellator_escape_cpp_string("${GIT_COMMIT}" TESSELLATOR_GIT_COMMIT_ESCAPED)
    tessellator_escape_cpp_string("${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}" TESSELLATOR_COMPILER_ESCAPED)
    tessellator_escape_cpp_string("${TESSELLATOR_BUILD_TYPE}" TESSELLATOR_BUILD_TYPE_ESCAPED)
    tessellator_escape_cpp_string("${CMAKE_CXX_FLAGS}" TESSELLATOR_COMPILATION_FLAGS_ESCAPED)
    tessellator_escape_cpp_string("${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}" TESSELLATOR_COMPILATION_FLAGS_DEBUG_ESCAPED)
    tessellator_escape_cpp_string("${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}" TESSELLATOR_COMPILATION_FLAGS_RELEASE_ESCAPED)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/tessellatorBuildInfo.h.in"
        "${CMAKE_CURRENT_BINARY_DIR}/tessellatorBuildInfo.h"
        @ONLY
    )

    if(WIN32)
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in"
            "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
            @ONLY
        )
        set(TESSELLATOR_WINDOWS_RESOURCE "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
    endif()

    add_library(tessellator-app
        "buildInfo.cpp"
        "vtkIO.cpp"
        "launcher.cpp"
    )

    target_include_directories(tessellator-app PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}"
    )

    find_package(Boost REQUIRED COMPONENTS program_options)

    find_package(nlohmann_json REQUIRED)

    target_link_libraries(tessellator-app
        ${VTK_LIBRARIES}
        Boost::program_options
        nlohmann_json::nlohmann_json
    )
    add_executable(tessellator
        "tessellator.cpp"
        ${TESSELLATOR_WINDOWS_RESOURCE}
    )

    target_link_libraries(tessellator tessellator-app tessellator-meshers)
    
else()
    add_library(tessellator-app INTERFACE)
endif()
