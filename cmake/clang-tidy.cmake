#  
#   This file adds custom target tidy that runs clang-tidy static analyzer 
#       Input variables:
#           - CLANG_TIDY_HEADER_FILES: Header files to be checked 
#               *Default: All .hpp, .h files in project source directory
#
#           - CLANG_TIDY_SOURCE_FILES: Source files to be checked
#               *Default: All .c, .cpp, .cxx files in project source directory
#
#           - CLANG_TIDY_EXTRA_ARGS: Extra clang tidy arguments
#               *Default: ""
#
#           - CLANG_TIDY_CHECKS: Enabled checks
#               *Default: -*,clang-analyzer-*,-clang-analyzer-cplusplus*
#
#           - CLANG_TIDY_SAVE_PATH: If enabled, saves clang-tidy output to CLANG_TIDY_SAVE_PATH
#
#   Output variables:
#       - CLANG_TIDY_EXE: Path to clang-tidy executable
#
#   TODO: Make a ADD_CLANG_TIDY_TARGET() function instead of taking header files manually
#   TODO: Check if this runs on Windows 
#   BUG: Subdirectory targets are not included in compile commands


# find clang-tidy
find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
)

if (NOT CLANG_TIDY_EXE)
	message(STATUS "clang-tidy not found")
else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")

    # Define default values of the input arguments
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    if (NOT CLANG_TIDY_HEADER_FILES)
        file(GLOB_RECURSE CLANG_TIDY_HEADER_FILES
            "${CMAKE_SOURCE_DIR}/*.h"
            "${CMAKE_SOURCE_DIR}/*.hpp"
        )
    endif()

    if (NOT CLANG_TIDY_SOURCE_FILES)
        file(GLOB_RECURSE CLANG_TIDY_SOURCE_FILES 
            "${CMAKE_SOURCE_DIR}/*.cpp"
            "${CMAKE_SOURCE_DIR}/*.cxx"
            "${CMAKE_SOURCE_DIR}/*.c"
        )
    endif()

    if (NOT CLANG_TIDY_CHECKS)
        set(CLANG_TIDY_CHECKS "-*,clang-analyzer-*,-clang-analyzer-cplusplus*")
    endif()

    if (NOT CLANG_TIDY_EXTRA_ARGS)
        set(CLANG_TIDY_EXTRA_ARGS "")
    endif()

    if (DEFINED CLANG_TIDY_SAVE_PATH)
        set (CLANG_TIDY_SAVE_ARGUMENT "> ${CLANG_TIDY_SAVE_PATH} >&1")
    endif()

    add_custom_target(
        clang-tidy
        COMMENT "Running clang-tidy:"
        COMMAND ${CLANG_TIDY_EXE}
            ${CLANG_TIDY_SOURCE_FILES}
            "-p=${CMAKE_BINARY_DIR}"
            "-checks=${CLANG_TIDY_CHECKS}"
            #"--header-filter=${CLANG_TIDY_HEADER_FILES}"
            "--analyze-temporary-dtors"
            ${CLANG_TIDY_SAVE_ARGUMENT}
            ${CLANG_TIDY_EXTRA_ARGS} 
    )
endif()
