
cmake_minimum_required(VERSION 2.6)

include(CheckFunctionExists)

if(WIN32)
    add_compile_options("-d2SSAOptimizer-")
endif()

project(neptun) #LANGUAGES CXX CUDA

if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CONFIGURATION_TYPES Debug Release)

include_directories(src)
include_directories(src/thirdparty)

# Disable unnnecesary GLFW stuff
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(src/thirdparty/glfw-3.2.1)

#set(glfw3_DIR ${NEPTUN_DEPS_STAGE_DIR}/glfw/lib/cmake/glfw3)

#find_package(glfw3 REQUIRED)

set (thirdparty_sources
    src/thirdparty/gl3w/include/GL/gl3w.h
    src/thirdparty/gl3w/include/GL/glcorearb.h
    src/thirdparty/gl3w/src/gl3w.c
    src/thirdparty/imgui/imconfig.h
    src/thirdparty/imgui/imgui.cpp
    src/thirdparty/imgui/imgui.h
    src/thirdparty/imgui/imgui_demo.cpp
    src/thirdparty/imgui/imgui_draw.cpp
    src/thirdparty/imgui/imgui_internal.h
    src/thirdparty/ImGuizmo/ImGuizmo.h
    src/thirdparty/ImGuizmo/ImGuizmo.cpp
    src/thirdparty/noc/noc_file_dialog.h
    src/thirdparty/tinyobjloader/tiny_obj_loader.h
    src/thirdparty/tetgen/tetgen.h
    src/thirdparty/tetgen/tetgen.cxx
    src/thirdparty/tetgen/predicates.cxx
)

source_group ("thirdparty\\" FILES
    ${thirdparty_sources}
)

#set (accelerator_sources
#    src/neptun/accelerators/accelerator.h
#    src/neptun/accelerators/accelerator.cpp
#)
#
#source_group ("accelerators\\" FILES
#    ${accelerator_sources}
#)
#
#set (math_sources
#    src/neptun/math/ray.h
#    src/neptun/math/ray.cpp
#)
#
#source_group ("math\\" FILES
#    ${math_sources}
#)

set (editor_sources
    src/neptun/editor/editor.h
    src/neptun/editor/editor.cpp
    src/neptun/editor/graphics.h
    src/neptun/editor/graphics.cpp
    src/neptun/editor/imgui_impl_glfw_gl3.h
    src/neptun/editor/imgui_impl_glfw_gl3.cpp
)

source_group ("editor\\" FILES
    ${editor_sources}
)

set (main_sources
    src/neptun/main/accelerator.cpp
    src/neptun/main/accelerator.h
    src/neptun/main/asset_importer.cpp
    src/neptun/main/asset_importer.h
    src/neptun/main/basis.h
    src/neptun/main/bounds.cpp
    src/neptun/main/bounds.h
    src/neptun/main/bvh.cpp
    src/neptun/main/bvh.h
    src/neptun/main/color.cpp
    src/neptun/main/color.h
    src/neptun/main/component.cpp
    src/neptun/main/component.h
    src/neptun/main/face.h
    src/neptun/main/hilbert.c
    src/neptun/main/hilbert.h
    src/neptun/main/image.cpp
    src/neptun/main/image.h
    src/neptun/main/kd_tree.cpp
    src/neptun/main/kd_tree.h
    src/neptun/main/light.cpp
    src/neptun/main/light.h
    src/neptun/main/logger.cpp
    src/neptun/main/logger.h
    src/neptun/main/main.cpp
    src/neptun/main/material.h
    src/neptun/main/memory.cpp
    src/neptun/main/memory.h
    src/neptun/main/mesh.cpp
    src/neptun/main/mesh.h
    src/neptun/main/proceduralmeshgenerator.cpp
    src/neptun/main/proceduralmeshgenerator.h
    src/neptun/main/ray.h
    src/neptun/main/ray_tracer.cpp
    src/neptun/main/ray_tracer.h
    src/neptun/main/scene.cpp
    src/neptun/main/scene.h
    src/neptun/main/sceneObject.cpp
    src/neptun/main/sceneObject.h
    src/neptun/main/sfc_utils.cpp
    src/neptun/main/sfc_utils.h
    src/neptun/main/stats.cpp
    src/neptun/main/stats.h
    src/neptun/main/tet_mesh.cpp
    src/neptun/main/tet_mesh.h
    src/neptun/main/texture.cpp
    src/neptun/main/texture.h
    src/neptun/main/utils.cpp
    src/neptun/main/utils.h
    src/neptun/main/filesystem.h
)

source_group ("main\\" FILES
    ${main_sources}
)

add_executable (neptun
    ${editor_sources}
    ${main_sources}
    ${thirdparty_sources}
)

# Find and link to OpenGL
include(FindOpenGL)
target_link_libraries(neptun ${OPENGL_gl_LIBRARY})

# Link to glfw
target_link_libraries(neptun glfw)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT neptun)

add_custom_command(
    TARGET neptun
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/src/neptun/editor/shaders" "${CMAKE_SOURCE_DIR}/sandbox/shaders"
    COMMENT "copying shaders"
    VERBATIM)

add_custom_command(
    TARGET neptun
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/src/thirdparty/imgui/extra_fonts" "${CMAKE_SOURCE_DIR}/sandbox/fonts"
    COMMENT "copying fonts"
    VERBATIM)

add_custom_command(
    TARGET neptun
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/src/neptun/editor/fonts" "${CMAKE_SOURCE_DIR}/sandbox/fonts"
    COMMENT "copying fonts"
    VERBATIM)

# Use _aligned_malloc if exists otherwise check posix_memalign
# Not tested on Windows
check_function_exists(_aligned_malloc HAVE_ALIGNED_MALLOC)
if (HAVE_ALIGNED_MALLOC)
    add_definitions(-DHAVE_ALIGNED_MALLOC)
else()
    check_function_exists(posix_memalign HAVE_POSIX_MEMALIGN)
    if (HAVE_POSIX_MEMALIGN)
        add_definitions(-DHAVE_POSIX_MEMALIGN)
    endif()
endif()

check_function_exists(snprintf HAVE_SNPRINTF)
if(HAVE_SNPRINTF)
    add_definitions(-DHAVE_SNPRINTF)
endif()

if(UNIX)
    if (APPLE)
        add_definitions(-DNOC_FILE_DIALOG_IMPLEMENTATION)
        add_definitions(-DNOC_FILE_DIALOG_OSX)
    else()
        #Noc requires gtk to run in linux
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(GTK3 REQUIRED gtk+-3.0) # Maybe use older gtk versions?

        if (GTK3_FOUND)
            include_directories(${GTK3_INCLUDE_DIRS})
            link_directories(${GTK3_LIBRARY_DIRS})
            add_definitions(${GTK3_CFLAGS_OTHER})
            target_link_libraries(neptun ${GTK3_LIBRARIES})

            add_definitions(-DNOC_FILE_DIALOG_GTK)
            add_definitions(-DNOC_FILE_DIALOG_IMPLEMENTATION)
        else()
            message( FATAL_ERROR "GTK is not found. GTK is required for linux builds.")
        endif()
    endif()

    set_target_properties(
        neptun
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/sandbox/bin/Release")
else() #Windows
    add_definitions(-DNOC_FILE_DIALOG_IMPLEMENTATION)
    add_definitions(-DNOC_FILE_DIALOG_WIN32)
    set_target_properties(
        neptun
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/sandbox/bin")
endif()

set_target_properties(
    neptun
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY  "${CMAKE_SOURCE_DIR}/sandbox/bin/$(Configuration)")
