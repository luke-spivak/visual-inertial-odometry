# Official generated C headers, pinned so wire definitions cannot drift by build.
# Offline builds may set VIO_MAVLINK_INCLUDE_DIR to this revision's extracted root.
set(VIO_MAVLINK_INCLUDE_DIR "" CACHE PATH "Root containing generated ardupilotmega/mavlink.h")
if(NOT VIO_MAVLINK_INCLUDE_DIR)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  include(FetchContent)
  FetchContent_Declare(mavlink_headers
    URL https://codeload.github.com/mavlink/c_library_v2/tar.gz/c1fd65eb702106097a4253c259aba34a18235848
    URL_HASH SHA256=25e5d57bf34ad4a0b28e5438e6a50e0608b48bb158cf989cef2d84383627d9b6)
  FetchContent_MakeAvailable(mavlink_headers)
  set(VIO_MAVLINK_INCLUDE_DIR "${mavlink_headers_SOURCE_DIR}")
endif()
if(NOT EXISTS "${VIO_MAVLINK_INCLUDE_DIR}/ardupilotmega/mavlink.h")
  message(FATAL_ERROR "VIO_MAVLINK_INCLUDE_DIR must contain ardupilotmega/mavlink.h")
endif()
add_library(vio_mavlink_headers INTERFACE)
target_include_directories(vio_mavlink_headers SYSTEM INTERFACE "${VIO_MAVLINK_INCLUDE_DIR}")
