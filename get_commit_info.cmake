set(GIT_COMMIT "UNKNOWN")

if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
    find_program(GIT_EXECUTABLE git)
    if(GIT_EXECUTABLE)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            OUTPUT_VARIABLE GIT_COMMIT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        if(NOT GIT_RESULT EQUAL 0 OR NOT GIT_COMMIT)
            set(GIT_COMMIT "UNKNOWN")
        endif()
    endif()
endif()

if(GIT_COMMIT STREQUAL "UNKNOWN" AND EXISTS "${CMAKE_SOURCE_DIR}/git_info.txt")
    file(READ "${CMAKE_SOURCE_DIR}/git_info.txt" GIT_INFO_CONTENT)
    string(REGEX MATCH "GIT_COMMIT=\"([^\"]+)\"" _ "${GIT_INFO_CONTENT}")
    if(CMAKE_MATCH_1)
        set(GIT_COMMIT "${CMAKE_MATCH_1}")
    endif()
endif()

message(STATUS "Tessellator Git commit: ${GIT_COMMIT}")
