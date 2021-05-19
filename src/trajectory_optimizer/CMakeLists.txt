cmake_minimum_required(VERSION 2.8.3)
PROJECT (trajectory_optimizer)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../libs/)

include_directories(../common/)
include_directories(../../deps/map/)
include_directories(../../deps/liblog/)
include_directories(../../deps/osqp/include)

link_directories(../../deps/map)
link_directories(../../deps/liblog)
link_directories(../../deps/osqp/lib)

SET(SRC_LIST  trajectory_planner.cpp 
              trajectory_planner.h
              path_planner.cpp
              path_planner.h
              cartesian_frenet_transform.cpp 
              cartesian_frenet_transform.h 
              obstacle_decider.cpp
              obstacle_decider.h
              path_optimizer_problem.cpp
              path_optimizer_problem.h
              optimizer_problem.cpp
              optimizer_problem.h
              speed_planner.cpp
              speed_planner.h 
              target_line.cpp
              target_line.h 
              path_boundary.cpp
              path_boundary.h 
              planning_visualization.cpp
              planning_visualization.h
              trajectory_generator.cpp 
              trajectory_generator.h 
              cubic_bezier_curve.cpp 
              cubic_bezier_curve.h 
              )


add_library(trajectory_optimizer SHARED ${SRC_LIST} )
target_link_libraries(trajectory_optimizer osqp pthread VTSMapInterfaceCPP)