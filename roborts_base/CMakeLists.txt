cmake_minimum_required(VERSION 2.8.3)

project(roborts_base)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roborts_msgs
  roscpp
  std_msgs
  tf
  actionlib
)

add_message_files(
        DIRECTORY msg
        FILES
        Bullet.msg
        GoalTask.msg
)


add_service_files(
        DIRECTORY srv
        FILES
        TestRefree.srv
)

add_action_files(
        DIRECTORY action
        FILES
        TurnAngle.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)


catkin_package(
)


# 通讯节点
add_executable(roborts_base_node
        serial_comm_node.cpp
        serial_comm_node.h
        turn_angle_action.cpp
        turn_angle_action.h
        writetxt.h
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware/serial_device.cpp
        )

target_link_libraries(roborts_base_node
        PUBLIC
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY})

target_include_directories(roborts_base_node
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware
        )
add_dependencies(roborts_base_node roborts_msgs_generate_messages)
add_dependencies(roborts_base_node
        roborts_base_gencpp)


# 测试turnangleaction节点
add_executable(test_turnangle_client_node
        test_turnangle_client.cpp
        )

target_link_libraries(test_turnangle_client_node
        PUBLIC
        ${catkin_LIBRARIES}
        )

target_include_directories(test_turnangle_client_node
        PUBLIC
        ${catkin_INCLUDE_DIRS})

add_dependencies(test_turnangle_client_node
        roborts_base_gencpp)

# 伪裁判系统节点
add_executable(test_refree_node
        test_refree_node.cpp
        test_refree_node.h
        )

target_link_libraries(test_refree_node
        PUBLIC
        ${catkin_LIBRARIES}
        )

target_include_directories(test_refree_node
        PUBLIC
        ${catkin_INCLUDE_DIRS})

add_dependencies(test_refree_node
        roborts_msgs_generate_messages)
add_dependencies(test_refree_node
        roborts_base_gencpp)