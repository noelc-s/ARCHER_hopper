set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "Configuring Autodiff Interface...")

set(AD_INC "include")
set(AD_HEADER_LIST
        "${AD_INC}/auto_diff_types.h"
        "${AD_INC}/cpp_ad_interface.h")

set(AD_SOURCE_LIST
        cpp_ad_interface.cpp)

add_library(Autodiff
        ${AD_SOURCE_LIST}
        ${AD_HEADER_LIST})

target_include_directories(Autodiff PUBLIC
    ${AD_INC})

# TODO: Move to a package manager
# For now we will get this stuff through pinocchio maybe
#target_link_libraries(Autodiff PUBLIC cppad)

#----------- Eigen ----------- #
find_package(Eigen3 REQUIRED)
target_link_libraries(Autodiff PUBLIC Eigen3::Eigen)

#----------- Pinocchio ----------- #
find_package(pinocchio 3.0.0 REQUIRED)
target_link_libraries(Autodiff PUBLIC pinocchio::pinocchio)

include_directories(/home/noelcs/repos/CppADCodeGen_install/include)
include_directories(/home/noelcs/repos/CppAD_install/include)