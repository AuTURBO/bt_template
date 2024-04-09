cmake_minimum_required(VERSION 3.8)
project(bt_sample)

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
ament_auto_find_build_dependencies()

######################################################

set(dependencies
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    std_msgs
    )

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(bt_node src/bt_node.cpp)
ament_target_dependencies(bt_node
  std_msgs
  ${dependencies}
)

######################################################


install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config include src launch srv log
  DESTINATION share/${PROJECT_NAME}
)

######################################################



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
