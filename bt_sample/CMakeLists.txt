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
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED )
find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

######################################################
# add executables
######################################################

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE src/bt_node.cpp)
target_include_directories(${PROJECT_NAME}
    PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_action
    std_msgs
    behaviortree_cpp
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
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config include src srv log
        DESTINATION share/${PROJECT_NAME})

ament_package()