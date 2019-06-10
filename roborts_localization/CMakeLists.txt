cmake_minimum_required(VERSION 2.8.3)

project(roborts_localization)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf
        roborts_msgs
        apriltags2_ros
        actionlib
)

add_action_files(
        DIRECTORY action
        FILES
        SupplyPid.action
        GlobalPlanner.action
        LocalPlanner.action
)

add_service_files(
        DIRECTORY srv
        FILES
        StartVision.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
        geometry_msgs
        nav_msgs
)


catkin_package()

add_subdirectory(amcl)

add_executable(localization_node
        localization_node.cpp
        localization_node.h
        localization_math.cpp
        writetxt.h
        )

target_link_libraries(localization_node
        PUBLIC
        amcl
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY})

target_include_directories(localization_node
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})
add_dependencies(localization_node roborts_msgs_generate_messages)


add_executable(supply_node_version
        supply_node_version2.cpp
        supply_node_version2.h
        writetxt.h
        )

target_link_libraries(supply_node_version
        PUBLIC
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY})

target_include_directories(supply_node_version
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})

add_dependencies(supply_node_version
        roborts_localization_gencpp)


add_executable(supply_node
        supply_node.cpp
        supply_node.h
        writetxt.h
        )

target_link_libraries(supply_node
        PUBLIC
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY})

target_include_directories(supply_node
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})

add_dependencies(supply_node
        roborts_localization_gencpp)


add_executable(test_client_node
        test_client_node.cpp
        )

target_link_libraries(test_client_node
        PUBLIC
        ${catkin_LIBRARIES}
        )

target_include_directories(test_client_node
        PUBLIC
        ${catkin_INCLUDE_DIRS})

add_dependencies(test_client_node
        roborts_localization_gencpp)
