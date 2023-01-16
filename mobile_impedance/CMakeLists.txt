cmake_minimum_required(VERSION 3.0.2)
project(mobile_impedance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mobile_impedance
 CATKIN_DEPENDS gazebo_msgs roscpp std_msgs geometry_msgs sensor_msgs
 DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/velocity.cpp
)


add_executable(impedance_force
  src/impedance_force.cpp
)
target_link_libraries(impedance_force ${catkin_LIBRARIES})
add_dependencies(impedance_force ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mobile_admittance
  src/mobile_admittance.cpp
)
target_link_libraries(mobile_admittance ${catkin_LIBRARIES})
add_dependencies(mobile_admittance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

