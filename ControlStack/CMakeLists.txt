cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

# two arguments to be passed in -- the path to where the dependencies are installed, and the path 
# to where the BezierTubes repository is located
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

# Option to build hardare support (requires ros to be installed, not needed for sim only results)
option(HARDWARE "build with hardware support (ros)" OFF)

# Find relevant packages
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(glfw3 REQUIRED )
find_package(pinocchio REQUIRED)
find_package(manif REQUIRED)
find_library(MUJOCO_LIB mujoco REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)
if(HARDWARE)
  find_package(roscpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
endif()

# Include relevant directories
include_directories(
	inc
  /usr/local/include
  /usr/local/include/manif 
  /usr/local/include/pinocchio
  /usr/local/include/mujoco
	/usr/local/include/eigen-cdd
)

# Bezier Tubes
set(bezier_DIR /home/docker/libs/BezierTubes)
add_library(bezier SHARED IMPORTED)
set_property(TARGET bezier PROPERTY IMPORTED_LOCATION ${bezier_DIR}/build/libBEZIER.so)
include_directories(bezier INTERFACE ${bezier_DIR}/inc)
add_library(cddeigen SHARED IMPORTED)
set_target_properties(cddeigen PROPERTIES IMPORTED_LOCATION /usr/local/lib/libeigen-cdd.so)

# Trajectory class
add_library(TRAJECTORY SHARED
    src/Trajectory.cpp
)
target_link_libraries(TRAJECTORY Eigen3::Eigen)
target_link_libraries(TRAJECTORY bezier)
target_link_libraries(TRAJECTORY cddeigen)

# MPC class
add_library(MPC SHARED
    src/Hopper.cpp
    src/MPC.cpp
)
target_link_libraries(MPC Eigen3::Eigen)
target_link_libraries(MPC osqp::osqp)
target_link_libraries(MPC OsqpEigen::OsqpEigen)
target_link_libraries(MPC ${PINOCCHIO_LIBRARIES})
target_link_directories(MPC PUBLIC ${PINOCCHIO_INCLUDE_DIRS})
target_link_libraries(MPC TRAJECTORY)

# Graph class
add_library(GRAPH SHARED
    src/Graph.cpp
)  
target_link_libraries(GRAPH Eigen3::Eigen)
target_link_libraries(GRAPH cddeigen) 

# Controller class
add_library(CONTROLLER SHARED
	src/Controller.cpp
) 
target_link_libraries(CONTROLLER Eigen3::Eigen)
target_link_libraries(CONTROLLER osqp::osqp)
target_link_libraries(CONTROLLER MPC)
target_link_libraries(CONTROLLER TRAJECTORY)
target_link_libraries(CONTROLLER yaml-cpp)
target_link_libraries(CONTROLLER GRAPH)
target_link_libraries(CONTROLLER python3.10)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(CONTROLLER ${PYTHON_LIBRARIES})

# Simulator class
add_library(SIMULATOR SHARED
  src/Simulator.cpp
) 
target_link_libraries(SIMULATOR Eigen3::Eigen)
target_link_libraries(SIMULATOR glfw)
target_link_libraries(SIMULATOR yaml-cpp)
target_link_libraries(SIMULATOR ${MUJOCO_LIB})
#target_link_libraries(SIMULATOR ${MUJOCO_DIR})
target_link_directories(SIMULATOR PUBLIC ${MUJOCO_PATH})
target_link_libraries(SIMULATOR python3.10)
#include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(SIMULATOR ${PYTHON_LIBRARIES})


############################## Executables #################################
############################################################################
add_executable(hopper_ctrl src/MpcSim.cpp)
target_link_libraries(hopper_ctrl CONTROLLER)
target_link_libraries(hopper_ctrl SIMULATOR)

#add_executable(hopper_sim src/MpcSim.cpp)
#target_link_libraries(hopper_sim SIMULATOR)

if(HARDWARE)
add_executable(hopper_ctrl_hardware src/Controller_Hardware.cpp)
target_link_libraries(hopper_ctrl_hardware MPC)
target_link_libraries(hopper_ctrl_hardware yaml-cpp)
target_include_directories(hopper_ctrl_hardware PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(hopper_ctrl_hardware ${roscpp_LIBRARIES})
endif()

################ Pybind11 #########################
add_subdirectory(pybind11)
pybind11_add_module(hopper "../bindings/hopper.cpp")
target_link_libraries(hopper PUBLIC Eigen3::Eigen)
target_link_libraries(hopper PUBLIC CONTROLLER)
target_link_libraries(hopper PUBLIC SIMULATOR)
target_link_libraries(hopper PUBLIC yaml-cpp)
