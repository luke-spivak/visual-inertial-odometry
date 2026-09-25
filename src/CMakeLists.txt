cmake_minimum_required(VERSION 3.16)
project(vio_application LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED NO_MODULE)
add_library(vio_core estimate.cpp frames.cpp)
target_include_directories(vio_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vio_core PUBLIC Eigen3::Eigen)
target_compile_options(vio_core PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_navigation ../tests/test_navigation.cpp)
  target_link_libraries(test_navigation PRIVATE vio_core)
  add_test(NAME navigation COMMAND test_navigation
           ${CMAKE_CURRENT_SOURCE_DIR}/../tests/data/navigation_cases.txt)
endif()
