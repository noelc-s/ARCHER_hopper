cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

if (NOT DEFINED PATH_TO_REPOS)
  set(PATH_TO_REPOS /home/sergio/repos)
endif()
if (NOT DEFINED PATH_TO_BEZIER)
  set(PATH_TO_BEZIER /home/sergio/repos)
endif()

SET(MUJOCO_DIR ${PATH_TO_REPOS}/mujoco_install CACHE PATH "Path to Mujoco dir")
SET(PINOCCHIO_DIR ${PATH_TO_REPOS}/pinocchio_install CACHE PATH "Path to pinocchio dir")
SET(MANIF_DIR ${PATH_TO_REPOS}/manif_install CACHE PATH "Path to manif dir")

list(APPEND CMAKE_PREFIX_PATH ${PINOCCHIO_DIR})
list(APPEND CMAKE_PREFIX_PATH ${PATH_TO_REPOS}/eigenpy_install)

option(HARDWARE "build with hardware support (ros)" OFF)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(osqp REQUIRED HINTS ${PATH_TO_REPOS}/osqp_install)
find_package(OsqpEigen REQUIRED HINTS ${PATH_TO_REPOS}/osqp-eigen_install)
find_package(glfw3 REQUIRED HINTS ${PATH_TO_REPOS}/glfw_install)
find_package(pinocchio REQUIRED HINTS ${PINOCCHIO_DIR}/lib)
find_package(manif REQUIRED HINTS ${MANIF_DIR})
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)
add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)

if(HARDWARE)
  find_package(roscpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
endif()

include_directories(
	inc
	${manif_INCLUDE_DIRS}
	${PINOCCHIO_DIR}/include
	${MUJOCO_DIR}/include
	${MUJOCO_DIR}/include/mujoco
	${PATH_TO_REPOS}/eigen-cddlib_install/include/eigen-cdd
)

# Bezier Tubes
set(bezier_DIR ${PATH_TO_BEZIER}/BezierTubes)
add_library(bezier SHARED IMPORTED)
set_property(TARGET bezier PROPERTY IMPORTED_LOCATION ${bezier_DIR}/build/libBEZIER.so)
include_directories(bezier INTERFACE ${bezier_DIR}/inc)
add_library(cddeigen SHARED IMPORTED)
set_target_properties(cddeigen PROPERTIES IMPORTED_LOCATION ${PATH_TO_REPOS}/eigen-cddlib_install/lib/libeigen-cdd.so)


add_library(TRAJECTORY SHARED
    src/Trajectory.cpp
)
target_link_libraries(TRAJECTORY Eigen3::Eigen)
target_link_libraries(TRAJECTORY bezier)
target_link_libraries(TRAJECTORY cddeigen)

add_library(MPC SHARED
    src/Hopper.cpp
    src/MPC.cpp
)
target_link_libraries(MPC Eigen3::Eigen)
target_link_libraries(MPC osqp::osqp)
target_link_libraries(MPC OsqpEigen::OsqpEigen)
target_link_libraries(MPC ${PINOCCHIO_LIBRARIES})
target_link_directories(MPC PUBLIC ${PINOCCHIO_INCLUDE_DIRS})
target_link_libraries(MPC TRAJECTORY)  ##############

add_library(GRAPH SHARED
    src/Graph.cpp
)  
target_link_libraries(GRAPH Eigen3::Eigen)  ##############
target_link_libraries(GRAPH cddeigen)  ##############

add_library(CONTROLLER SHARED
	src/Controller.cpp
) 
target_link_libraries(CONTROLLER Eigen3::Eigen)
target_link_libraries(CONTROLLER osqp::osqp)


add_executable(hopper_ctrl src/MpcSim.cpp)
target_link_libraries(hopper_ctrl MPC)
target_link_libraries(hopper_ctrl CONTROLLER)
target_link_libraries(hopper_ctrl TRAJECTORY)
target_link_libraries(hopper_ctrl yaml-cpp)
target_link_libraries(hopper_ctrl GRAPH)  ##############

if(HARDWARE)
add_executable(hopper_ctrl_hardware src/Controller_Hardware.cpp)
target_link_libraries(hopper_ctrl_hardware MPC)
target_link_libraries(hopper_ctrl_hardware yaml-cpp)
target_include_directories(hopper_ctrl_hardware PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(hopper_ctrl_hardware ${roscpp_LIBRARIES})
endif()

add_executable(hopper_sim src/Simulator.cpp)
target_link_libraries(hopper_sim PUBLIC Eigen3::Eigen)
target_link_libraries(hopper_sim PUBLIC glfw)
target_link_libraries(hopper_sim PUBLIC yaml-cpp)
target_link_libraries(hopper_sim PUBLIC ${MUJOCO_LIB})         ###############################
target_link_directories(hopper_sim PUBLIC ${MUJOCO_PATH})


