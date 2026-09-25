# vio_live -- links against libov_msckf_lib.so from a ROS-free OpenVINS build.
# Built by tools/deploy/build_vio.sh using tools/buildenv.
cmake_minimum_required(VERSION 3.16)
project(vio_live CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(OV_SRC "" CACHE PATH "open_vins source tree")
set(OV_LIB "" CACHE FILEPATH "libov_msckf_lib.so built from OV_SRC")
if (NOT EXISTS "${OV_SRC}/ov_msckf/src/core/VioManager.h" OR NOT EXISTS "${OV_LIB}")
    message(FATAL_ERROR "set OV_SRC and OV_LIB to matching OpenVINS source and library")
endif ()

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)

# Must match the library's definitions: TrackAruco.h changes shape on this
# flag, and a mismatch is a silent layout difference, not a link error.
add_definitions(-DENABLE_ARUCO_TAGS=0)

if(NOT TARGET vio_recording)
    find_package(Threads REQUIRED)
    add_library(vio_recording ../recording.cpp)
    target_include_directories(vio_recording PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_link_libraries(vio_recording PUBLIC Threads::Threads)
endif()
add_library(vio_openvins sensor_runner.cpp)
add_executable(vio_live legacy_main.cpp)
target_link_libraries(vio_live PRIVATE vio_openvins)
target_include_directories(vio_openvins PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OV_SRC}/ov_core/src ${OV_SRC}/ov_init/src ${OV_SRC}/ov_msckf/src
    ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(vio_openvins PUBLIC vio_recording ${OV_LIB} ${OpenCV_LIBRARIES}
    Boost::system Boost::filesystem Boost::thread Boost::date_time ${CERES_LIBRARIES} pthread)
# Libraries the Pi lacks ship beside the binary in lib/.
set_target_properties(vio_live PROPERTIES BUILD_RPATH "\$ORIGIN/lib")
