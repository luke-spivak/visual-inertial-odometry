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

include(cmake/Mavlink.cmake)
add_library(vio_flight_link flight_session.cpp serial_port.cpp mavlink_link.cpp)
target_link_libraries(vio_flight_link PUBLIC vio_core vio_mavlink_headers)
target_compile_options(vio_flight_link PRIVATE -Wall -Wextra -Wpedantic)

find_package(nlohmann_json 3.11 REQUIRED)
add_library(vio_imu_device imu_device.cpp)
target_include_directories(vio_imu_device PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vio_imu_device PUBLIC nlohmann_json::nlohmann_json)
target_compile_options(vio_imu_device PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_imu_device ../tests/test_imu_device.cpp)
  target_link_libraries(test_imu_device PRIVATE vio_imu_device)
  add_test(NAME imu_device COMMAND test_imu_device)
  find_package(Threads REQUIRED)
  add_executable(test_serial_port ../tests/test_serial_port.cpp)
  target_link_libraries(test_serial_port PRIVATE vio_flight_link Threads::Threads)
  add_test(NAME serial_port COMMAND test_serial_port)
  add_executable(test_mavlink_link ../tests/test_mavlink_link.cpp)
  target_link_libraries(test_mavlink_link PRIVATE vio_flight_link)
  add_test(NAME mavlink_link COMMAND test_mavlink_link
    ${CMAKE_CURRENT_SOURCE_DIR}/../results/flight-review-2026-09-15/pi/run-20260915-121608.est.txt)

  add_executable(test_navigation ../tests/test_navigation.cpp)
  target_link_libraries(test_navigation PRIVATE vio_core)
  add_test(NAME navigation COMMAND test_navigation
           ${CMAKE_CURRENT_SOURCE_DIR}/../tests/data/navigation_cases.txt)
  set_tests_properties(imu_device serial_port mavlink_link navigation PROPERTIES TIMEOUT 30)
endif()
