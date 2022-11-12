cmake_minimum_required(VERSION 3.14)
project(mobile_robot)

find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs sensor_msgs std_msgs tf)

catkin_python_setup()

add_action_files(
        DIRECTORY action
        FILES
        MissionPlan.action
        Movement.action
        Rotation.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp actionlib rospy message_generation std_msgs message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_compass src/imu_compass.cpp)
target_link_libraries(imu_compass ${catkin_LIBRARIES})

install(
  TARGETS imu_compass
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
        scripts/gps_to_xy.py
        scripts/mission_planner.py
        scripts/odom_to_yaw.py
        scripts/moving_average_heading_filter.py
        scripts/sg_heading_filter.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )