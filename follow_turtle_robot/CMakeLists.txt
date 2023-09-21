cmake_minimum_required(VERSION 3.8)
project(follow_turtle_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_spawner src/turtle_spawner.cpp) # create executable, 
ament_target_dependencies(turtle_spawner rclcpp turtlesim my_robot_interfaces) # dependencies for this node

add_executable(robot src/robot.cpp)
ament_target_dependencies(robot rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS
  turtle_spawner
  robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
