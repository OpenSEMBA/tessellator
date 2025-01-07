message(STATUS "Creating build system for tessellator-app")

add_library(tessellator-app
    "VTKParser.cpp"
)

find_package(VTK COMPONENTS 
  CommonCore
#   FiltersSources
  IOGeometry
  IOLegacy
  IOXML
)

target_link_libraries(tessellator-app
    ${VTK_LIBRARIES}
)