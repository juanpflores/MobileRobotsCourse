cmake_minimum_required(VERSION 2.8.3)
project(arms)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  message_generation
  hardware_tools #In this package is located the Roboclaw.py
)

catkin_package(
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )


add_executable(left_arm src/left_arm.cpp)
add_executable(right_arm src/right_arm.cpp)

add_dependencies(left_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(right_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(left_arm
  ${catkin_LIBRARIES}
)
target_link_libraries(right_arm
  ${catkin_LIBRARIES}
)
      
