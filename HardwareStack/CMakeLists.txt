cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper_low_level)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 REQUIRED)
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manif REQUIRED)

add_executable(hopper src/Hopper.cpp)
target_link_libraries(hopper Eigen3::Eigen)
target_include_directories(hopper PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(hopper ${roscpp_LIBRARIES})
target_include_directories(hopper PUBLIC ${manif_INCLUDE_DIRS})
