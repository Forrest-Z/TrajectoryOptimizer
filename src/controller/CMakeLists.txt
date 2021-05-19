cmake_minimum_required(VERSION 2.8.3)
PROJECT (controller)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../libs/)

include_directories(../common/)
include_directories(../../deps/map/)
# include_directories(../../deps/liblog/)

link_directories(../../deps/map)
link_directories(../../deps/liblog)

SET(SRC_LIST  controller.cpp 
              controller.h 
              pid_controller.cpp
              pid_controller.h       
              )


add_library(controller SHARED ${SRC_LIST} )
target_link_libraries(controller VTSMapInterfaceCPP)