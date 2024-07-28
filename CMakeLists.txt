cmake_minimum_required(VERSION 3.8)
project(behaviour_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_PREFIX_PATH "/usr/local/include")
# list(APPEND CMAKE_MODULE_PATH "ros-projects/dev_pi_communicate/install/behaviortree_ros2/share/behaviortree_ros2/cmake")
# list(APPEND CMAKE_MODULE_PATH "/usr/local/share/behaviortree_cpp/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)  ##
find_package(nav2_behavior_tree REQUIRED)  ##
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_pkg REQUIRED)
find_package(oakd_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# find_package(my_bt REQUIRED)
# find_package(behaviortree_ros2 REQUIRED)


# Install directories
install(DIRECTORY
bt_xml launch config
DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOUR_SOURCES
src/GoToBallPose.cpp
src/GoToSiloPose.cpp
src/fib_behaviour.cpp
src/GoTo.cpp
src/GoToOrigin.cpp
src/GoToMiddle.cpp
src/isBallDetected.cpp
src/spinActionClient.cpp
src/waitActionClient.cpp
src/PacketPublisher.cpp
src/InitiallizeActuators.cpp
src/LineFollower.cpp
src/ResetOdom.cpp
src/BackUpActionClient.cpp
src/RecoveryNode.cpp
src/StartAndWait.cpp 
src/ZoneAndColorConfig.cpp
src/CvConfig.cpp

src/actions/TurnOnRoller.cpp
src/actions/TurnOnConveyer.cpp
src/actions/TurnOffRoller.cpp
src/actions/TurnOffConveyer.cpp
src/actions/PneumaticOff.cpp
src/actions/PneumaticOn.cpp

src/decorator/returnFailureNode.cpp
src/decorator/returnSuccessNode.cpp
src/decorator/waitMilliseconds.cpp

src/conditions/isBallInside.cpp
src/conditions/isOnlyBall.cpp

src/control/pipeline_sequence.cpp
src/control/recovery_node.cpp
src/control/round_robin_node.cpp
)
# add_library(
#   include/robotlibpc/cpp/crypto/crc8.cpp
#   include/robotlibpc/cpp/controller/pid.cpp
#   include/robotlibpc/cpp/communication/serial.cpp
#   include/robotlibpc/cpp/msgs/state_msg.cpp
# )

set(TARGET_DEPENDS
      rclcpp
      rclcpp_action
      nav2_msgs
      nav2_behaviors  ##
      nav2_behavior_tree
      behaviortree_cpp_v3
      yaml-cpp
      tf2_geometry_msgs
      tf2
      action_pkg
      oakd_msgs
      lifecycle_msgs
      # behaviortree_ros2
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

include_directories(
  include 
)

add_executable(bt src/bt.cpp ${BEHAVIOUR_SOURCES})
target_link_libraries(bt ${YAML_CPP_LIBRARIES} BT::behaviortree_cpp_v3)
# target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)
ament_target_dependencies(bt ${TARGET_DEPENDS})

install(TARGETS
bt
DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Test.action"
)

ament_package()

