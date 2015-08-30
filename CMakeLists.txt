cmake_minimum_required (VERSION 2.8.5)
project(VisualizerTest)

include(GNUInstallDirs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
find_package(OpenSim)

add_subdirectory(src)
