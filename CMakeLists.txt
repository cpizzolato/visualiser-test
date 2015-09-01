cmake_minimum_required (VERSION 2.8.5)
project(VisualizerTest)

include(GNUInstallDirs)

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++1y   -Wno-deprecated-declarations -pthread")
endif(UNIX)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
find_package(OpenSim)

add_subdirectory(src)
