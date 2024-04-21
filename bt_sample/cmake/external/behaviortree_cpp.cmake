find_package(behaviortree_cpp CONFIG)

if(NOT behaviortree_cpp_FOUND)
    message(STATUS "Downloading BehaviorTree.CPP")
    include(FetchContent)

    set(BTCPP_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
    set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "Build commandline tools" FORCE)
    set(BTCPP_EXAMPLES    OFF CACHE BOOL "Build tutorials and examples" FORCE)
    set(BTCPP_UNIT_TESTS  OFF CACHE BOOL "Build the unit tests" FORCE)

    set(FETCHCONTENT_QUIET OFF)
    FetchContent_Declare(behavior_tree_cpp
        GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
        GIT_TAG        4.5.2
        GIT_PROGRESS   TRUE
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(behavior_tree_cpp)
endif()