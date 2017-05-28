cmake_minimum_required(VERSION 2.8.3)
project(prisme_gazebo)


find_package(
    catkin REQUIRED
    COMPONENTS gazebo gazebo_plugins gazebo_ros roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}/Paging # workaround for: "OgrePagedWorldSection.h: No such file or directory"
)

add_library(imu_gazebo src/gazebo_ros_imu_sensor.cpp)
add_library(light_sensor_gazebo src/light_sensor_plugin.cpp)

target_link_libraries(light_sensor_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin )

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
