cmake_minimum_required(VERSION 2.6)

project(neptun)
include_directories(src/thirdparty)

set (accelerator_sources
    src/neptun/accelerators/accelerator.h
    src/neptun/accelerators/accelerator.cpp
)

source_group ("accelerators\\" FILES
    ${accelerator_sources}
)

set (main_sources
    src/neptun/main/main.cpp
)

source_group ("main\\" FILES
    ${main_sources}
)

add_executable(neptun ${main_sources} ${accelerator_sources})

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT neptun)