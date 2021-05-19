cmake_minimum_required(VERSION 2.8.3)
PROJECT (target_generator)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../libs/)

include_directories(../common/)
include_directories(../../deps/map/)
include_directories(../../deps/liblog/)

link_directories(../../deps/map)
link_directories(../../deps/liblog)

SET(SRC_LIST  local_target_generator.cpp 
              local_target_generator.h 
              ../../deps/liblog/log_manager.cpp 
              ../../deps/liblog/log_manager.h               
              )



add_library(target_generator SHARED ${SRC_LIST} )
target_link_libraries(target_generator vtslog pthread VTSMapInterfaceCPP)