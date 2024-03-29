cmake_minimum_required(VERSION 3.8)
project(behaviour_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/home/rpi/ros-projects/dev_pi_communicate/install/action_pkg")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_pkg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install directories
install(DIRECTORY
bt_xml launch config
DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOUR_SOURCES
src/navigation_behaviour.cpp
src/fib_behaviour.cpp
)

set(TARGET_DEPENDS
      rclcpp
      rclcpp_action
      nav2_msgs
      behaviortree_cpp
      yaml-cpp
      tf2_geometry_msgs
      tf2
      action_pkg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include action_pkg/include/action/)
add_executable(bt src/bt.cpp ${BEHAVIOUR_SOURCES})
ament_target_dependencies(bt ${TARGET_DEPENDS})
target_link_libraries(bt ${YAML_CPP_LIBRARIES})

install(TARGETS
bt
DESTINATION lib/${PROJECT_NAME}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Test.action"
  # "action/Action.action"
)



ament_package()
