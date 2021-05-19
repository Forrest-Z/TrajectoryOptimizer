cmake_minimum_required(VERSION 2.8.3)
PROJECT (planning_module)
SET(CMAKE_BUILD_TYPE "Release")
set(TOP_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(${TOP_DIR}/src/common)
add_subdirectory(${TOP_DIR}/src/target_generator)
add_subdirectory(${TOP_DIR}/src/trajectory_optimizer)
add_subdirectory(${TOP_DIR}/src/controller)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TOP_DIR}/)
# SET(CMAKE_BUILD_TYPE DEBUG)
SET(SRC_LIST ${TOP_DIR}/main.cpp 
             ${TOP_DIR}/src/common/header.h
             ${TOP_DIR}/deps/liblog/log_manager.cpp 
             ${TOP_DIR}/deps/liblog/log_manager.h 
             )

include_directories(${TOP_DIR}/deps/map/)
include_directories(${TOP_DIR}/deps/liblog/)
include_directories(${TOP_DIR}/deps/libdrive/include/)
include_directories(/usr/include/python3.6/)
include_directories(${TOP_DIR}/src/common/)
include_directories(${TOP_DIR}/src/target_generator/)
include_directories(${TOP_DIR}/src/trajectory_optimizer/)
include_directories(${TOP_DIR}/src/controller/)

link_directories(${TOP_DIR}/deps/map)
link_directories(${TOP_DIR}/deps/liblog)
link_directories(${TOP_DIR}/deps/libdrive/lib)
link_directories(/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu)


add_executable(planning ${SRC_LIST})
target_link_libraries(planning  target_generator
                                trajectory_optimizer
                                controller
                                adapter_gflags
                                protobuf protobuf-lite protoc Drive vtslog pthread MulticastNetwork python3.6)