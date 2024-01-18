cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

set(PATH_TO_REPOS /home/noelcs/repos)

# Option to build hardare support (requires ros to be installed, not needed for sim only results)
option(HARDWARE "build with hardware support (ros)" OFF)

# Set pat dependencies (can be set manually if they are in different locations)
SET(MUJOCO_DIR ${PATH_TO_REPOS}/mujoco_install CACHE PATH "Path to Mujoco dir")
SET(PINOCCHIO_DIR ${PATH_TO_REPOS}/pinocchio_install CACHE PATH "Path to pinocchio dir")
SET(MANIF_DIR ${PATH_TO_REPOS}/manif_install CACHE PATH "Path to manif dir")
list(APPEND CMAKE_PREFIX_PATH ${PINOCCHIO_DIR})
list(APPEND CMAKE_PREFIX_PATH ${PATH_TO_REPOS})
list(APPEND CMAKE_PREFIX_PATH ${PATH_TO_REPOS}/eigenpy_install)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED HINTS ${PATH_TO_REPOS}/osqp-eigen_install)
find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED HINTS ${PINOCCHIO_DIR}/lib)
find_package(manif REQUIRED HINTS ${MANIF_DIR})
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${PINOCCHIO_DIR}/include)

include_directories(${MUJOCO_DIR}/include)
include_directories(${MUJOCO_DIR}/include/mujoco)
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)

add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)

add_library(MPC SHARED
    src/Hopper.cpp
    src/MPC.cpp
)
target_link_libraries(MPC Eigen3::Eigen)
target_link_libraries(MPC osqp::osqp)
target_link_libraries(MPC OsqpEigen::OsqpEigen)
target_include_directories(MPC PUBLIC ${manif_INCLUDE_DIRS})
target_link_libraries(MPC ${PINOCCHIO_LIBRARIES})
target_link_directories(MPC PUBLIC ${PINOCCHIO_INCLUDE_DIRS})

# add the executable
add_executable(hopper_ctrl src/Controller.cpp)
target_link_libraries(hopper_ctrl MPC)
target_link_libraries(hopper_ctrl yaml-cpp)

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
target_link_libraries(hopper_sim PUBLIC ${MUJOCO_LIB})
target_link_directories(hopper_sim PUBLIC ${MUJOCO_PATH})
