
cmake_minimum_required(VERSION 2.6)

include(CheckFunctionExists)

if(WIN32)
    #add_compile_options("-d2SSAOptimizer-")
    add_compile_options("-Zi")
    #add_compile_options("/arch:AVX2")
    add_compile_definitions(NOMINMAX)
endif()

if(WIN32)
    project(neptun LANGUAGES C CXX CUDA)
else()
    project(neptun LANGUAGES C CXX)
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O3")
endif()

if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo)
#SET(USE_STATIC_RUNTIME)

include_directories(src)
include_directories(src/thirdparty)

if(UNIX)
    # Ideally, We should use config dependent path here.
    SET(embree_DIR "${NEPTUN_DEPS_STAGE_DIR}/linux/embree/lib/cmake/embree-3.4.0")
else()
    SET(embree_DIR "${NEPTUN_DEPS_STAGE_DIR}/embree-release")
endif()


FIND_PACKAGE(embree 3.4 REQUIRED)
INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})

#include glfw
option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
add_subdirectory(src/thirdparty/glfw-3.2.1)

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
    src/thirdparty/tinyxml2/tinyxml2.h
    src/thirdparty/tinyxml2/tinyxml2.cpp
)

source_group ("thirdparty\\" FILES
    ${thirdparty_sources}
)

#TODO: SORT ME
set (neptun2_sources
    src/neptun2/accelerators/accelerator.cpp
    src/neptun2/accelerators/accelerator.h
    src/neptun2/accelerators/bvh_embree.h
    src/neptun2/accelerators/bvh_embree.cpp
    src/neptun2/accelerators/noaccelerator.cpp
    src/neptun2/accelerators/noaccelerator.h
    src/neptun2/accelerators/ray.h
    src/neptun2/integrators/integrator.h
    src/neptun2/integrators/cast.cpp
    src/neptun2/integrators/cast.h
    src/neptun2/main/image.cpp
    src/neptun2/main/image.h
    src/neptun2/main/scene.cpp
    src/neptun2/main/scene.h
    src/neptun2/main/scene_object.cpp
    src/neptun2/main/scene_object.h
    src/neptun2/math/transform.cpp
    src/neptun2/math/transform.h
    src/neptun2/main/mesh.cpp
    src/neptun2/main/mesh.h
    src/neptun2/main/material.h
    src/neptun2/main/sceneio.cpp
    src/neptun2/main/sceneio.h
    src/neptun2/util/filesystem.h
    src/neptun2/util/parallel.h
    src/neptun2/util/timer.h
)

source_group ("neptun2_sources\\" FILES
    ${neptun2_sources}
)

set (cuda_sources
    src/neptun/cuda/test1.cu
)

source_group ("cuda\\" FILES
    ${cuda_sources}
)

set (math_sources
    src/neptun/math/matrix.h
    src/neptun/math/matrix.cpp
    src/neptun/math/transform.cpp
    src/neptun/math/transform.h
)

source_group ("math\\" FILES
    ${math_sources}
)

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
    src/neptun/argparse/argparse.cpp
    src/neptun/argparse/argparse.h
    src/neptun/argparse/argument.cpp
    src/neptun/argparse/argument.h
    src/neptun/argparse/util.h
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
    src/neptun/main/filesystem.h
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
    src/neptun/main/procedural_mesh_generator.cpp
    src/neptun/main/procedural_mesh_generator.h
    src/neptun/main/ray_tracer.cpp
    src/neptun/main/ray_tracer.h
    src/neptun/main/ray.h
    src/neptun/main/scene.cpp
    src/neptun/main/scene.h
    src/neptun/main/sceneobject.cpp
    src/neptun/main/sceneobject.h
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
    src/neptun/tet_mesh/tet_mesh_builder_factory.h
    src/neptun/tet_mesh/tet_mesh_builder.h
    src/neptun/tet_mesh/tetgen_tet_mesh_builder.h
    src/neptun/util/timer.h
)

source_group ("main\\" FILES
    ${main_sources}
)

add_executable (neptun
    #${cuda_sources}
    ${neptun2_sources}
    ${editor_sources}
    ${main_sources}
    ${math_sources}
    ${thirdparty_sources}
)

#Link & include embree
if(UNIX)
    target_link_libraries(neptun ${EMBREE_LIBRARY})
else()
    target_link_libraries(neptun
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/lexers.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/math.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/simd.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/sys.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/tasking.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/embree3.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/embree_sse42.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/embree_avx.lib
        debug       ${NEPTUN_DEPS_STAGE_DIR}/embree-debug/lib/embree_avx2.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/lexers.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/math.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/simd.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/sys.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/tasking.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/embree3.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/embree_sse42.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/embree_avx.lib
        optimized   ${NEPTUN_DEPS_STAGE_DIR}/embree-release/lib/embree_avx2.lib)
endif()

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
    set_property(TARGET neptun APPEND PROPERTY LINK_FLAGS /DEBUG)
    
endif()

set_target_properties(
    neptun
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY  "${CMAKE_SOURCE_DIR}/sandbox/bin/$(Configuration)")
