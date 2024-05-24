cmake_minimum_required(VERSION 3.8)
project(bt_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################################################
# find dependencies
######################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(behaviortree_cpp REQUIRED )
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

######################################################

set(dependencies
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    std_msgs
    ament_index_cpp
    nav2_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    yaml-cpp
    )

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)


set(AUTONOMY_SOURCES
    src/bt_tree.cpp 
)
######################################################
add_executable(bt_main src/bt_main.cpp ${AUTONOMY_SOURCES})
ament_target_dependencies(bt_main
  ${dependencies}
)
target_link_libraries(bt_main yaml-cpp)

######################################################

add_executable(nav2_feedback_listener src/nav2_feedback_listener.cpp)
ament_target_dependencies(nav2_feedback_listener
  rclcpp
  rclcpp_action
  nav2_msgs
)

install(TARGETS
  bt_main
  nav2_feedback_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config include src launch srv log maps
  DESTINATION share/${PROJECT_NAME}
)

######################################################



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
