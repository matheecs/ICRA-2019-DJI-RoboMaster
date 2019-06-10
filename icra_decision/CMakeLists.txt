cmake_minimum_required(VERSION 3.1)
project(icra_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  actionlib
  tf
  roborts_costmap
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msgs
  FILES
  EnemyDetect.msg
  GoalTask.msg
  RobotDamage.msg
  RobotStatus.msg
  BonusStatus.msg
  GameResult.msg
  GameStatus.msg
  GameSurvivor.msg
  RobotDamage.msg
  RobotHeat.msg
  RobotShoot.msg
  RobotStatus.msg
  SupplierStatus.msg
  Bullet.msg
)

add_service_files(
  DIRECTORY msgs
  FILES
  ChassisMode.srv
  GimbalMode.srv
)

add_action_files(
  DIRECTORY msgs
  FILES
  LocalPlanner.action
  GlobalPlanner.action
  SupplyPid.action
  TurnAngle.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(icra_decision
  src/decision.cpp
)
target_link_libraries(icra_decision
  ${catkin_LIBRARIES}
  roborts_costmap
)
add_dependencies(icra_decision
  icra_decision_generate_messages_cpp
)