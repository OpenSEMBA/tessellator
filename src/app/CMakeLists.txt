message(STATUS "Creating build system for tessellator-app")

add_library(tessellator-app
    "vtkIO.cpp"
)

find_package(VTK COMPONENTS 
  CommonCore
  IOGeometry
  IOLegacy
  IOXML
)

target_link_libraries(tessellator-app
    ${VTK_LIBRARIES}
)