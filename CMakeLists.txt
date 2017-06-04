cmake_minimum_required(VERSION 2.6)

set(GLM_ROOT_DIR "" CACHE STRING "GLM root directory")

project(neptun)
include_directories(${GLM_ROOT_DIR})
add_executable(neptun src/neptun/main.cpp)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT neptun)