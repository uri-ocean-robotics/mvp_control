cmake_minimum_required(VERSION 3.10)
project(mvp_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

set(dependencies
  rclcpp
  nav_msgs
  tf2_ros
  mvp_msgs
  tf2_eigen
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

set(library_name ${PROJECT_NAME})


include(FetchContent)
# OSQP
if (NOT TARGET osqp-cpp)
  set (PYTHON OFF CACHE INTERNAL "Turn off python for OSQP-CPP")
  message(STATUS "${PROJECT_NAME}: `osqp-cpp` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    osqp-cpp
    GIT_REPOSITORY https://github.com/GSO-soslab/osqp-cpp
    GIT_TAG        stable
  )
  FetchContent_MakeAvailable(osqp-cpp)
else()
  message(STATUS "osqp-cpp: `osqp` targets found.")
endif()


#Add library
add_library(${library_name} SHARED 
  src/mvp_control_ros_node.cpp
  src/mvp_control/polynomial_solver.cpp
  src/mvp_control/mimo_pid.cpp
  src/mvp_control/thruster_ros.cpp
  src/mvp_control/vector_thruster_ros.cpp
  src/mvp_control/mvp_control.cpp
  src/mvp_control/mvp_control_ros.cpp
)

target_link_libraries( 
  ${library_name}
  gsl
  osqp-cpp
  ${Boost_LIBRARIES}
  yaml-cpp
)


ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
  )

add_executable(mvp_control_ros_node src/mvp_control_ros_node.cpp)
target_link_libraries(mvp_control_ros_node 
  ${library_name}
  gsl
  osqp-cpp
  ${Boost_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(mvp_control_ros_node ${dependencies})

# add_executable(yaml_test src/yaml_test.cpp)
# target_link_libraries(yaml_test 
#   ${library_name}
#   gsl
#   ${Boost_LIBRARIES}
#   yaml-cpp
# )
# ament_target_dependencies(yaml_test ${dependencies})



# install(TARGETS yaml_test ${library_name}
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )


install(TARGETS mvp_control_ros_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()