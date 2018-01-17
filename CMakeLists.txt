cmake_minimum_required(VERSION 2.8.3)
project(controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros_tools
    trajectory_generators
    cmake_modules
    roscpp
    rtt_ros
    rtt_roscomm
    rtt_rosclock
    eigen_conversions
    kdl_conversions
    trac_ik_lib
    tf
    rtt_sensor_msgs
    rtt_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
      include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

# Orocos typekits
ros_generate_rtt_typekit(controllers)

set(COMPONENT_SOURCES
    src/chainfksolveracc_recursive.cpp
    src/pid_joint_space.cpp
    src/pid_task_space.cpp
    src/computed_torque_joint_space.cpp
    src/operational_space_control.cpp
)

# Orocos component
orocos_component(${PROJECT_NAME} ${COMPONENT_SOURCES})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
)

## Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
