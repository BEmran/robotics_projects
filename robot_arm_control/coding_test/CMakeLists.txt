cmake_minimum_required(VERSION 3.5)
project(coding_test)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(coding_test_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(dummy_joint_controller src/DummyJointController.cpp include/coding_test/DummyJointController.hpp src/dummy_joint_controller_main.cpp test/test_common.hpp)
target_include_directories(dummy_joint_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dummy_joint_controller
        rclcpp
        sensor_msgs
        )

add_executable(high_level_controller src/HighLevelController.cpp include/coding_test/HighLevelController.hpp src/high_level_controller_main.cpp)
target_include_directories(high_level_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(high_level_controller
        rclcpp
        rclcpp_action
        coding_test_msgs
        )

add_executable(my_dummy_joint_controller
        src/MyDummyJointController.cpp
        include/coding_test/MyDummyJointController.hpp
        src/my_dummy_joint_controller_main.cpp
        test/test_common.hpp
        src/utils.cpp
        include/coding_test/utils.hpp
        )

target_include_directories(my_dummy_joint_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(my_dummy_joint_controller
        rclcpp
        rclcpp_action
        sensor_msgs
        coding_test_msgs
        )

add_executable(path_planner src/PathPlanner.cpp
        include/coding_test/PathPlanner.hpp
        src/path_planner_main.cpp
        )

target_include_directories(path_planner PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(path_planner
        rclcpp
        rclcpp_action
        sensor_msgs
        coding_test_msgs
        )

add_executable(joint_observer src/JointObserver.cpp
        include/coding_test/JointObserver.hpp
        src/joint_observer_main.cpp
        src/utils.cpp
        include/coding_test/utils.hpp
        )

target_include_directories(joint_observer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(joint_observer
        rclcpp
        rclcpp_action
        sensor_msgs
        geometry_msgs
        coding_test_msgs
        tf2_ros
        )

install(TARGETS dummy_joint_controller high_level_controller my_dummy_joint_controller path_planner joint_observer
        DESTINATION lib/${PROJECT_NAME})

install(FILES

        config/simple_robot.launch.py
        config/my_simple_robot.launch.py
        config/simple_robot.urdf
        config/config.rviz

        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_dummy_joint_controller test/test_dummy_joint_controller.cpp src/DummyJointController.cpp)
    ament_target_dependencies(test_dummy_joint_controller rclcpp sensor_msgs)
    target_include_directories(test_dummy_joint_controller PUBLIC include)

    ament_add_gtest(test_high_level_controller test/test_high_level_controller.cpp src/HighLevelController.cpp)
    ament_target_dependencies(test_high_level_controller rclcpp rclcpp_action coding_test_msgs)
    target_include_directories(test_high_level_controller PUBLIC include)
endif ()

ament_package()
