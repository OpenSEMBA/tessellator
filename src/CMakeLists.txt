include_directories(".")

add_subdirectory(utils)
add_subdirectory(core)
add_subdirectory(meshers)

if (TESSELLATOR_ENABLE_CGAL)
    add_subdirectory(cgal)
endif()

add_subdirectory(app)