cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread -fopenmp")

set(PATH_TO_REPOS /home/noelcs/repos CACHE PATH "Path to repos")
message(${PATH_TO_REPOS})

# Option to build hardare support (requires ros to be installed, not needed for sim only results)
option(HARDWARE "build with hardware support (ros)" ON)
option(MPC "build with MPC controller" ON)
option(LEARNING "build with NN controller" ON)

# Set pat dependencies (can be set manually if they are in different locations)
SET(MUJOCO_DIR ${PATH_TO_REPOS}/mujoco_install CACHE PATH "Path to Mujoco dir")
SET(PINOCCHIO_DIR ${PATH_TO_REPOS}/pinocchio_install CACHE PATH "Path to pinocchio dir")
SET(MANIF_DIR ${PATH_TO_REPOS}/manif_install CACHE PATH "Path to manif dir")
SET(onnxruntime_DIR ${PATH_TO_REPOS}/onnxruntime_install CACHE PATH "Path to onnx dir")

list(APPEND CMAKE_PREFIX_PATH ${PINOCCHIO_DIR} ${PATH_TO_REPOS} ${PATH_TO_REPOS}/eigenpy_install)


find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED HINTS ${PINOCCHIO_DIR}/lib)
find_package(manif REQUIRED HINTS ${MANIF_DIR})
if(MPC)
endif()
if(HARDWARE)
    find_package(roscpp REQUIRED)
    find_package(geometry_msgs REQUIRED)
endif()
if(LEARNING)
    find_package(onnxruntime REQUIRED HINTS ${onnxruntime_DIR}/lib/cmake/onnxruntime)
endif()

### OSQP
find_package(osqp)
if (NOT TARGET osqpstatic)  # This is what osqp-cpp looks for
    message(STATUS "osqpstatic target not found, checking osqp::osqpstatic...")
    if (NOT TARGET osqp::osqpstatic)
        message(STATUS "Could not find osqp::osqpstatic, osqp-cpp will try to fetch it...")
    else ()
        message(STATUS "Found osqp::osqpstatic. Using it in osqp-cpp.")
        add_library(osqpstatic ALIAS osqp::osqpstatic)
    endif ()
endif ()

include(FetchContent)

set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL "Build OSQP-CPP without tests")
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "Variable for OSQP-CPP ABSL dep")

FetchContent_Declare(
        osqp-cpp
        GIT_REPOSITORY https://github.com/google/osqp-cpp.git
        GIT_TAG 43433736334d6b515ea4b0247156fea9e56c0d3f
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(osqp-cpp)
###

message(${MUJOCO_DIR})
include_directories(${MUJOCO_DIR}/include)
include_directories(${MUJOCO_DIR}/include/mujoco)
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)

add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)

# add the executable
add_executable(hopper_ctrl src/Controller.cpp src/utils.cpp src/Hopper.cpp src/Policy.cpp src/UserInput.cpp src/rom.cpp)
target_link_libraries(hopper_ctrl yaml-cpp)

target_link_libraries(hopper_ctrl osqp-cpp)
target_link_libraries(hopper_ctrl pinocchio::pinocchio)
# message(${PINOCCHIO_LIBRARIES})
# target_link_directories(hopper_ctrl PUBLIC ${PINOCCHIO_INCLUDE_DIRS})
if(LEARNING)
    target_link_libraries(hopper_ctrl onnxruntime::onnxruntime)
    target_include_directories(hopper_ctrl PUBLIC ${onnxruntime_DIR}/include)
endif()

if(MPC)
    # add_library(MPC SHARED
    #     src/MPC.cpp
    # )
    target_link_libraries(hopper_ctrl Eigen3::Eigen)
    target_include_directories(hopper_ctrl PUBLIC ${manif_INCLUDE_DIRS})
    # target_link_directories(hopper_ctrl PUBLIC ${PINOCCHIO_INCLUDE_DIRS})
    # target_link_libraries(hopper_ctrl MPC)
    if(LEARNING)
        target_link_libraries(hopper_ctrl onnxruntime::onnxruntime)
        target_include_directories(hopper_ctrl PUBLIC ${onnxruntime_DIR}/include)
    endif()
endif()

if(HARDWARE)
    message(STATUS "Hardware is building")
    add_executable(hopper_ctrl_hardware src/Controller_Hardware.cpp src/utils.cpp src/Hopper.cpp src/Policy.cpp src/UserInput.cpp src/rom.cpp)
    target_link_libraries(hopper_ctrl_hardware yaml-cpp)
    target_link_libraries(hopper_ctrl_hardware Eigen3::Eigen)
    target_include_directories(hopper_ctrl_hardware PUBLIC ${roscpp_INCLUDE_DIRS})
    target_include_directories(hopper_ctrl_hardware PUBLIC ${manif_INCLUDE_DIRS})
    target_link_libraries(hopper_ctrl_hardware ${roscpp_LIBRARIES})
    if(LEARNING)
        target_link_libraries(hopper_ctrl_hardware onnxruntime::onnxruntime)
        target_include_directories(hopper_ctrl_hardware PUBLIC ${onnxruntime_DIR}/include)
    endif()
    # if(MPC)
    #     target_link_libraries(hopper_ctrl_hardware MPC)
    # endif()
endif()

add_executable(hopper_sim src/Simulator.cpp)
target_link_libraries(hopper_sim PUBLIC Eigen3::Eigen)
target_link_libraries(hopper_sim PUBLIC glfw)
target_link_libraries(hopper_sim PUBLIC yaml-cpp)
target_link_libraries(hopper_sim PUBLIC ${MUJOCO_LIB})
target_link_directories(hopper_sim PUBLIC ${MUJOCO_PATH})

add_executable(hopper_sim_hardware src/SimVisHardware.cpp)
target_link_libraries(hopper_sim_hardware PUBLIC Eigen3::Eigen)
target_link_libraries(hopper_sim_hardware PUBLIC glfw)
target_link_libraries(hopper_sim_hardware PUBLIC yaml-cpp)
target_link_libraries(hopper_sim_hardware PUBLIC ${MUJOCO_LIB})
target_link_directories(hopper_sim_hardware PUBLIC ${MUJOCO_PATH})
