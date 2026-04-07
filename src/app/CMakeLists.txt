message(STATUS "Creating build system for tessellator-app")

# Configure version resource file for Windows
if(WIN32)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/version.rc.in"
        "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
        @ONLY
    )
    set(WINDOWS_RC_FILE "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
else()
    set(WINDOWS_RC_FILE "")
endif()

add_library(tessellator-app
    "vtkIO.cpp"
    "launcher.cpp"
    ${WINDOWS_RC_FILE}
)

find_package(VTK COMPONENTS 
  CommonCore
  IOGeometry
  IOLegacy
  FiltersCore
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
    ${WINDOWS_RC_FILE}
)

target_link_libraries(tessellator tessellator-app tessellator-meshers)
