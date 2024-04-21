cmake_minimum_required(VERSION 3.8)
project(bt_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################################################
# find dependencies
######################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include(cmake/external/behaviortree_cpp.cmake)

######################################################
# add executables
######################################################
add_executable(bt_node)
target_sources(bt_node
    PRIVATE src/bt_node.cpp)
target_include_directories(bt_node
    PRIVATE include)
target_link_libraries(bt_node
    PRIVATE behaviortree_cpp)
ament_target_dependencies(bt_node
    PUBLIC  rclcpp
            rclcpp_action
            std_msgs
            ament_index_cpp)

######################################################
# test
######################################################
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

######################################################
# install
######################################################
install(TARGETS bt_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config include src srv log
        DESTINATION share/${PROJECT_NAME})

ament_package()