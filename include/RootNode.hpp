#ifndef RESET_ODOM_HPP
#define RESET_ODOM_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

// #include "geometry_msgs/msg/pose.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "action_pkg/msg/robot_config.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class RootNode : public BT::SyncActionNode
{
    public:
    RootNode(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<action_pkg::msg::RobotConfig>> subscription_;


    // // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(action_pkg::msg::RobotConfig);
    
};

#endif