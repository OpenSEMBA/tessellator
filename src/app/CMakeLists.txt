message(STATUS "Creating build system for tessellator-app")

if(TESSELLATOR_LOAD_APP)
    find_package(VTK COMPONENTS
        CommonCore
        IOGeometry
        IOLegacy
        FiltersCore
    )

    add_library(tessellator-app
        "vtkIO.cpp"
        "launcher.cpp"
    )

    find_package(Boost COMPONENTS program_options)

    find_package(nlohmann_json)

    target_link_libraries(tessellator-app
        ${VTK_LIBRARIES}
        Boost::program_options
        nlohmann_json::nlohmann_json
    )
    add_executable(tessellator
        "tessellator.cpp"
    )

    target_link_libraries(tessellator tessellator-app tessellator-meshers)
    
    else()
    add_library(tessellator-app INTERFACE)
endif()
