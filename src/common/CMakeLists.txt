cmake_minimum_required(VERSION 2.8.3)
PROJECT (ADAPTERGFLAGS)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../libs/)


include_directories(/usr/local/include/gflags)
include_directories(../../deps/map/)

link_directories(/usr/local/lib/)
link_directories(../../deps/map)


SET(SRC_LIST adapter_gflags.cc
             adapter_gflags.h
             header.h
             planning_struct.h
             st_graph_point.cpp 
             st_graph_point.h )

add_library(adapter_gflags SHARED ${SRC_LIST} )
target_link_libraries(adapter_gflags gflags VTSMapInterfaceCPP)