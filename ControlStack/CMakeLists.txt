cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread -fopenmp")

set(PATH_TO_REPOS /home/noelcs/repos CACHE PATH "Path to repos")
message("Looking for dependecies in: ${PATH_TO_REPOS}")

# Option to build hardare support 
option(HARDWARE "build with hardware support" ON)
option(MPC "build with MPC controller" OFF)
option(LEARNING "build with NN controller" OFF)
option(MUJOCO "build with Mujoco sim/vis" ON)
option(PINOCCHIO "build with pinocchio" OFF)
option(OPTITRACK "build with mocap" OFF)
option(PATHPLANNING "build with high level path planner" ON)

# Set pat dependencies (can be set manually if they are in different locations)
SET(MUJOCO_DIR ${PATH_TO_REPOS}/mujoco_install CACHE PATH "Path to Mujoco dir")
SET(MANIF_DIR ${PATH_TO_REPOS}/manif_install CACHE PATH "Path to manif dir")
list(APPEND CMAKE_PREFIX_PATH ${PATH_TO_REPOS})

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(manif REQUIRED HINTS ${MANIF_DIR})

# Mujoco
if(MUJOCO)
find_package(glfw3 REQUIRED)
include_directories(${MUJOCO_DIR}/include)
include_directories(${MUJOCO_DIR}/include/mujoco)
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)
add_library(mujoco SHARED IMPORTED GLOBAL)
set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${CMAKE_INCLUDE_DIR}/lib/libmujoco.so)
endif()

include_directories(${PATH_TO_REPOS}/librealsense-2.53.1/include)
add_library(realsense2 SHARED IMPORTED)
set_property(TARGET realsense2 PROPERTY IMPORTED_LOCATION ${PATH_TO_REPOS}/librealsense-2.53.1/build/librealsense2.so)

if (PATHPLANNING)
    # SET(MPC ON)
    SET(PLANNER_DIR ${PATH_TO_REPOS}/PathPlanner CACHE PATH "Path to PathPlanner dir")
    set(bezier_DIR ${PATH_TO_REPOS}/BezierTubes/cpp)
    include_directories(bezier INTERFACE ${bezier_DIR}/inc)
    include_directories(${PLANNER_DIR}/inc)

    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    include_directories(${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} 
                        ${nav_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
                        ${tf2_ros_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS}
                        ${visualization_msgs_INCLUDE_DIRS})
    message(${tf2_INCLUDE_DIRS})
    message(${tf2_geometry_msgs_INCLUDE_DIRS})
    find_package(local_mapper_interfaces REQUIRED)

    include(FetchContent)
    FetchContent_Declare(
        graph
        SOURCE_DIR ${PLANNER_DIR}
    )
    FetchContent_MakeAvailable(graph)

    add_library(PLANNER SHARED src/planner/planner_wrapper.cpp src/planner/ros_subscriber.cpp)
    target_link_libraries(PLANNER graph)
    target_link_libraries(PLANNER ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} 
                                    ${nav_msgs_LIBRARIES} ${tf2_LIBRARIES} 
                                    ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
                                    ${visualization_msgs_LIBRARIES})
    target_include_directories(PLANNER PUBLIC ${manif_INCLUDE_DIRS})
    ament_target_dependencies(PLANNER local_mapper_interfaces)
else()
    add_library(PLANNER SHARED src/planner/planner_dummy.cpp)
endif()
target_link_libraries(PLANNER Eigen3::Eigen)

if(PINOCCHIO)
    SET(PINOCCHIO_DIR ${PATH_TO_REPOS}/pinocchio_install CACHE PATH "Path to pinocchio dir")
    list(APPEND CMAKE_PREFIX_PATH ${PINOCCHIO_DIR} ${PATH_TO_REPOS}/eigenpy_install)
    find_package(pinocchio REQUIRED HINTS ${PINOCCHIO_DIR}/lib)
    add_library(PINOCCHIO SHARED src/pinocchio/pinocchio_wrapper.cpp)
    target_link_libraries(PINOCCHIO pinocchio::pinocchio)
    target_compile_definitions(PINOCCHIO PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")
else()
    add_library(PINOCCHIO SHARED src/pinocchio/pinocchio_dummy.cpp)
endif()
target_link_libraries(PINOCCHIO Eigen3::Eigen)

if(MPC)
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

    add_library(MPC SHARED src/mpc/mpc_wrapper.cpp)
    target_link_libraries(MPC osqp-cpp)
    target_link_libraries(MPC PINOCCHIO)
    
    target_include_directories(MPC PUBLIC ${manif_INCLUDE_DIRS})

    # target_compile_definitions(hopper_ctrl PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")

    # Add Google Test as a dependency
    include(FetchContent)
    FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # # Enable testing
    # enable_testing()
    # # Add subdirectory for tests
    # add_subdirectory(tests)
    # add_subdirectory(auto_diff)

else()
    add_library(MPC STATIC src/mpc/mpc_dummy.cpp)
endif()
target_link_libraries(MPC Eigen3::Eigen)

if(LEARNING)
    SET(onnxruntime_DIR ${PATH_TO_REPOS}/onnxruntime_install CACHE PATH "Path to onnx dir")
    find_package(onnxruntime REQUIRED HINTS ${onnxruntime_DIR}/lib/cmake/onnxruntime)
    target_link_libraries(hopper_ctrl onnxruntime::onnxruntime)
    target_include_directories(hopper_ctrl PUBLIC ${onnxruntime_DIR}/include)

    add_library(NN SHARED src/nn/nn_wrapper.cpp)

else()
    add_library(NN SHARED src/nn/nn_dummy.cpp)
endif()
target_link_libraries(NN Eigen3::Eigen)

# add the executable
add_executable(hopper_ctrl src/Controller.cpp src/utils.cpp src/Hopper.cpp src/Policy.cpp src/UserInput.cpp src/rom.cpp)
target_include_directories(hopper_ctrl PUBLIC ${manif_INCLUDE_DIRS})
target_link_libraries(hopper_ctrl PUBLIC yaml-cpp Eigen3::Eigen)
target_link_libraries(hopper_ctrl PUBLIC NN)
target_link_libraries(hopper_ctrl PUBLIC MPC)
target_link_libraries(hopper_ctrl PUBLIC PINOCCHIO)
target_link_libraries(hopper_ctrl PUBLIC PLANNER)

if(MUJOCO)
add_executable(hopper_sim src/Simulator.cpp)
target_link_libraries(hopper_sim PUBLIC Eigen3::Eigen glfw yaml-cpp)
target_link_libraries(hopper_sim PUBLIC ${MUJOCO_LIB})
target_link_directories(hopper_sim PUBLIC ${MUJOCO_PATH})
endif()

if(HARDWARE)
    message(STATUS "Hardware is building")
    
    add_executable(hopper_ctrl_hardware src/Controller_Hardware.cpp src/utils.cpp src/Hopper.cpp src/Policy.cpp src/UserInput.cpp src/rom.cpp)
    target_link_libraries(hopper_ctrl_hardware PUBLIC yaml-cpp)
    target_link_libraries(hopper_ctrl_hardware PUBLIC Eigen3::Eigen)
    target_include_directories(hopper_ctrl_hardware PUBLIC ${manif_INCLUDE_DIRS})
    if(MUJOCO)
      add_executable(hopper_sim_hardware src/SimVisHardware.cpp)
      target_link_libraries(hopper_sim_hardware PUBLIC Eigen3::Eigen)
      target_link_libraries(hopper_sim_hardware PUBLIC glfw)
      target_link_libraries(hopper_sim_hardware PUBLIC yaml-cpp)
      target_link_libraries(hopper_sim_hardware PUBLIC ${MUJOCO_LIB})
      target_link_directories(hopper_sim_hardware PUBLIC ${MUJOCO_PATH})
    endif()

    target_link_libraries(hopper_ctrl_hardware PUBLIC NN)
    target_link_libraries(hopper_ctrl_hardware PUBLIC MPC)
    target_link_libraries(hopper_ctrl_hardware PUBLIC PINOCCHIO)
    target_link_libraries(hopper_ctrl_hardware PUBLIC PLANNER)

    target_link_libraries(hopper_ctrl_hardware PUBLIC realsense2)

    if(OPTITRACK)
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        include_directories(${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

        add_library(OT SHARED src/optitrack/ot_wrapper.cpp)
        target_link_libraries(OT ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
    else()
        add_library(OT SHARED src/optitrack/ot_dummy.cpp)
    endif()
    target_link_libraries(OT Eigen3::Eigen)
    target_link_libraries(hopper_ctrl_hardware PUBLIC OT)

endif()


# cmake .. -DCMAKE_INSTALL_PREFIX=/home/noelcs/repos/eigenpy_install -DPYTHON_EXECUTABLE=/usr/bin/python3.8
