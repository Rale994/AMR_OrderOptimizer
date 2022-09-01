cmake_minimum_required(VERSION 3.10)
# project name
project(amr_ex_wts)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Wextra)

# One could use cmake to find all the required packages.
# find_package(GTest REQUIRED)

set(amr_INCLUDES
  include/amr_interface.hpp
  include/amr_task_executors.hpp
  include/amr_unit.hpp
  include/amr.hpp
  include/basic_structs.hpp)

set(amr_SOURCES
  src/amr_interface.cpp 
  src/amr_task_executors.cpp
  src/amr_unit.cpp
  src/basic_routines.cpp)

set(amr_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(amr_basis STATIC ${amr_SOURCES})
target_include_directories(amr_basis PUBLIC ${amr_INCLUDE_DIRS})
target_link_libraries(amr_basis PUBLIC
  -lyaml-cpp )

# executables are listed here:
add_executable( OrderOptimizer src/executables/amr_main.cpp )
target_link_libraries( OrderOptimizer PUBLIC amr_basis)
target_link_libraries( OrderOptimizer PUBLIC -lmosquitto -lyaml-cpp pthread )

#for google tests:
include(GoogleTest)

add_executable( RunAmrTests tests/amr_tests.cpp )
target_link_libraries( RunAmrTests PUBLIC amr_basis gtest pthread)

gtest_discover_tests(RunAmrTests
                TEST_SUFFIX .noArgs
                TEST_LIST   noArgsTests)
set_tests_properties(${noArgsTests}   PROPERTIES TIMEOUT 10)
