cmake_minimum_required(VERSION 3.1)
project(leonard_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Glog REQUIRED)

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



add_message_files(
  DIRECTORY msg
  FILES
  EnemyDetect.msg
  GoalTask.msg
  Bullet.msg
  TwistAccel.msg
)

add_message_files(
        DIRECTORY msg/referee_system
        FILES
        BonusStatus.msg
        GameResult.msg
        GameStatus.msg
        GameSurvivor.msg
        ProjectileSupply.msg
        RobotBonus.msg
        RobotDamage.msg
        RobotHeat.msg
        RobotShoot.msg
        RobotStatus.msg
        SupplierStatus.msg
)



add_service_files(
  DIRECTORY msg
  FILES
  ChassisMode.srv
  GimbalMode.srv
)

add_action_files(
  DIRECTORY msg
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

add_executable(leonard_decision
  src/decision.cpp
)
target_link_libraries(leonard_decision
        ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
  roborts_costmap
)
add_dependencies(leonard_decision
  leonard_decision_generate_messages_cpp
)

