cmake_minimum_required(VERSION 2.6)

project(neptun)
include_directories(src/thirdparty)
add_executable(neptun src/neptun/main.cpp)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT neptun)