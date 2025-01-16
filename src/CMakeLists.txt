include_directories(".")

add_subdirectory(utils)

if (TESSELLATOR_ENABLE_CGAL)
    add_subdirectory(cgal)
endif()

add_subdirectory(core)
add_subdirectory(meshers)
add_subdirectory(app)