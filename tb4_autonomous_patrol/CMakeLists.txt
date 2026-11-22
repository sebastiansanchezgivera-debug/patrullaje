cmake_minimum_required(VERSION 3.8)
project(tb4_autonomous_patrol)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# Compila en Stage sin Create 3; activa /dock y /undock cuando los
# mensajes oficiales estan instalados en el equipo del TurtleBot4.
find_package(irobot_create_msgs QUIET)
find_package(ament_index_cpp REQUIRED)

add_executable(patrol_runner
  src/main.cpp
  src/mission_supervisor.cpp
  src/safe_recovery.cpp
  src/dock_controller.cpp
)
target_include_directories(patrol_runner PRIVATE include)
target_link_libraries(patrol_runner Threads::Threads)
ament_target_dependencies(patrol_runner
  rclcpp rclcpp_action nav2_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs
  std_msgs behaviortree_cpp ament_index_cpp
)

if(irobot_create_msgs_FOUND)
  ament_target_dependencies(patrol_runner irobot_create_msgs)
  target_compile_definitions(patrol_runner PRIVATE TB4_HAS_CREATE3)
  message(STATUS "Integracion Create 3 disponible: Dock/Undock")
else()
  message(STATUS "Sin irobot_create_msgs: solo acoplamiento simulado")
endif()

install(TARGETS patrol_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/emergency_from_rviz.py scripts/prepare_nav2_safety.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
