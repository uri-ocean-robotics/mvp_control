cmake_minimum_required(VERSION 3.10)
project(mvp_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(dependencies
  rclcpp
  nav_msgs
  tf2_ros
  mvp_msgs
  tf2_eigen
  std_msgs
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)

find_package(GSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})


include(FetchContent)
# OSQP
if (NOT TARGET osqp-cpp)
  set (PYTHON OFF CACHE INTERNAL "Turn off python for OSQP-CPP")
  message(STATUS "${PROJECT_NAME}: `osqp-cpp` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    osqp-cpp
    GIT_REPOSITORY https://github.com/google/osqp-cpp/
    GIT_TAG        origin/master
  )
  FetchContent_MakeAvailable(osqp-cpp)
else()
  message(STATUS "osqp-cpp: `osqp` targets found.")
endif()


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_ros_node
        src/mvp_control_ros_node.cpp
        src/mvp_control/polynomial_solver.cpp
        src/mvp_control/mimo_pid.cpp
        src/mvp_control/thruster_ros.cpp
        src/mvp_control/mvp_control.cpp
        src/mvp_control/mvp_control_ros.cpp
)

target_link_libraries(${PROJECT_NAME}_ros_node
  osqp-cpp
  gsl
)

ament_target_dependencies(${PROJECT_NAME}_ros_node
  ${dependencies}
)

ament_package()