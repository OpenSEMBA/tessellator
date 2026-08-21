message(STATUS "Creating build system for tessellator-app")

if(TESSELLATOR_ENABLE_APP)
    if(WIN32)
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in"
            "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
            @ONLY
        )
        set(TESSELLATOR_WINDOWS_RESOURCE "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
    endif()

    add_library(tessellator-app
        "vtkIO.cpp"
        "launcher.cpp"
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
