cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_platform_crazyflie)
project(${PROJECT_NAME})

# Default to C++17 (required by rclcpp from Jazzy on)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
#########################


# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

set(PROJECT_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


#download the crazyflie_cpp from github
set(USE_CRAZYFLIE_COMPLETE_LIB OFF)
find_package(crazyflie_cpp QUIET)
if(${crazyflie_cpp_FOUND})
  message(STATUS "crazyflie_cpp found at ${crazyflie_cpp_DIR}")
  find_package(crazyflie_cpp REQUIRED)
  set(USE_CRAZYFLIE_COMPLETE_LIB ON)
else()
  message(STATUS "crazyflie_cpp not found")
  # crazyflie-link-cpp's Findlibusb.cmake only looks for a static libusb-1.0.a
  # on macOS, which conda-forge and Homebrew do not ship by default. Seed its
  # cache variables so the shared library is found instead.
  if(APPLE)
    find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0)
    find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
  endif()
  include(FetchContent)
  fetchcontent_declare(
    crazyflie_cpp
    GIT_REPOSITORY https://github.com/miferco97/crazyflie_cpp.git
    # Pinned for reproducible builds: master as of 2022-11-15.
    GIT_TAG b1aac13077633f7b505ed6c8029d587f18dece70
    # Only crazyflie-link-cpp is needed; its own submodules (libusb, pybind11)
    # are for Windows and the Python bindings.
    GIT_SUBMODULES crazyflie-link-cpp
    GIT_SUBMODULES_RECURSE OFF
  )
  # Build the fetched libraries static and link them into the nodes: the
  # package installs only its executables, and conda/colcon builds set
  # BUILD_SHARED_LIBS=ON, which would leave uninstalled shared libraries and
  # drop crazyflie_cpp's PRIVATE link to crazyflieLinkCpp from the nodes.
  set(_as2_cf_build_shared_libs ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  fetchcontent_makeavailable(crazyflie_cpp)
  target_compile_options(crazyflieLinkCpp PRIVATE -include string)
  set(BUILD_SHARED_LIBS ${_as2_cf_build_shared_libs})
endif()

set(HEADER_HPP_FILES
  include/crazyflie_platform.hpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/crazyflie_node.cpp
  src/crazyflie_platform.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  ${PROJECT_DEPENDENCIES}
)

if(${USE_CRAZYFLIE_COMPLETE_LIB})
  target_link_libraries(${PROJECT_NAME}_node crazyflie_cpp::crazyflie_cpp)
else()
  target_link_libraries(${PROJECT_NAME}_node crazyflie_cpp)
endif()


add_executable(${PROJECT_NAME}_swarm_node
  src/crazyflie_swarm_nodes.cpp
  src/crazyflie_platform.cpp)

ament_target_dependencies(${PROJECT_NAME}_swarm_node
  ${PROJECT_DEPENDENCIES}
)

if(${USE_CRAZYFLIE_COMPLETE_LIB})
  target_link_libraries(${PROJECT_NAME}_swarm_node
    crazyflie_cpp::crazyflie_cpp
  )
else()
target_link_libraries(${PROJECT_NAME}_swarm_node
  crazyflie_cpp
)
endif()



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
${PROJECT_NAME}_node
${PROJECT_NAME}_swarm_node
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/aideck_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
