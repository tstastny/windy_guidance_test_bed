cmake_minimum_required(VERSION 2.8.12)
project(cpp_functions)

find_package( PkgConfig )
find_package( GTest REQUIRED )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
find_package( pybind11 REQUIRED )

include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories( ${GTEST_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR} src include )

# ######
# pybind
# ######

pybind11_add_module(cpp_functions
    src/cpp_python_interface.cpp
    src/helpers.cpp
    src/npfg.cpp
    src/path_type.cpp)

# #####
# Tests
# #####

enable_testing()

add_executable(test_path test/test_path_type.cpp)

# add path.cpp to a testing lib to be linked to the testing executable
add_library(test_lib src/path_type.cpp include/path_type.h)

# link the gtest and project libs
target_link_libraries(test_path gtest pthread test_lib)

# This is so you can do 'make test' to see all your tests run, instead of
# manually running the executable test_xxx to see those specific tests.
#add_test(UnitTests runUnitTests)