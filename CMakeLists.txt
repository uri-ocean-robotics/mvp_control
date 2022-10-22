cmake_minimum_required(VERSION 3.0.2)
project(mvp_control)

# add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  dynamic_reconfigure
  mvp_msgs
)

find_package(GSL REQUIRED)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system regex)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/dynamic_reconfigure_pid.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mvp_control
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs mvp_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
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
  ${catkin_INCLUDE_DIRS}
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

add_dependencies(${PROJECT_NAME}_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ros_node
  osqp-cpp
  gsl
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mvp_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)