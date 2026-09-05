foreach(required_variable
    TESSELLATOR_EXECUTABLE
    TESSELLATOR_RUNTIME_DIRECTORY
    TESSELLATOR_STAGING_DIRECTORY
    TESSELLATOR_LICENSE
    TESSELLATOR_README)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
        message(FATAL_ERROR "${required_variable} must be provided")
    endif()
endforeach()

foreach(path_variable
    TESSELLATOR_EXECUTABLE
    TESSELLATOR_RUNTIME_DIRECTORY
    TESSELLATOR_LICENSE
    TESSELLATOR_README)
    if(NOT EXISTS "${${path_variable}}")
        message(FATAL_ERROR "${path_variable} does not exist: ${${path_variable}}")
    endif()
endforeach()

file(MAKE_DIRECTORY "${TESSELLATOR_STAGING_DIRECTORY}")

# Resolve only the DLLs used by the executable (and their transitive
# dependencies), rather than packaging every DLL installed by vcpkg.
file(GET_RUNTIME_DEPENDENCIES
    EXECUTABLES "${TESSELLATOR_EXECUTABLE}"
    RESOLVED_DEPENDENCIES_VAR resolved_dependencies
    UNRESOLVED_DEPENDENCIES_VAR unresolved_dependencies
    DIRECTORIES "${TESSELLATOR_RUNTIME_DIRECTORY}"
    PRE_EXCLUDE_REGEXES
        "^api-ms-win-.*"
        "^ext-ms-win-.*"
    # Keep the Visual C++ runtime self-contained even when it is resolved from
    # the Windows directory. All other Windows system DLLs are intentionally
    # left to the operating system.
    POST_INCLUDE_REGEXES
        ".*[/\\\\][Vv][Cc][Rr][Uu][Nn][Tt][Ii][Mm][Ee].*\\.dll$"
        ".*[/\\\\][Mm][Ss][Vv][Cc][Pp].*\\.dll$"
        ".*[/\\\\][Cc][Oo][Nn][Cc][Rr][Tt].*\\.dll$"
    POST_EXCLUDE_REGEXES
        "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\].*")

if(unresolved_dependencies)
    list(JOIN unresolved_dependencies "\n  " unresolved_dependencies_text)
    message(FATAL_ERROR "Unable to resolve required runtime dependencies:\n  ${unresolved_dependencies_text}")
endif()

file(COPY
    "${TESSELLATOR_EXECUTABLE}"
    "${TESSELLATOR_LICENSE}"
    "${TESSELLATOR_README}"
    DESTINATION "${TESSELLATOR_STAGING_DIRECTORY}")

foreach(dependency IN LISTS resolved_dependencies)
    file(COPY "${dependency}" DESTINATION "${TESSELLATOR_STAGING_DIRECTORY}")
endforeach()

list(LENGTH resolved_dependencies dependency_count)
message(STATUS "Staged tessellator.exe and ${dependency_count} runtime DLLs in ${TESSELLATOR_STAGING_DIRECTORY}")
