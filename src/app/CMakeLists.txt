message(STATUS "Creating build system for tessellator-app")

find_package(VTK COMPONENTS 
  CommonCore
  IOGeometry
  IOLegacy
  FiltersCore
)

if(VTK_FOUND)
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
    message(STATUS "VTK not found - tessellator app will not be built")
    add_library(tessellator-app INTERFACE)
endif()
