#ifndef IS_BALL_DETECTED_H
#define IS_BALL_DETECTED_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
class isBallDetected : public BT::SyncActionNode
{
    public:
    isBallDetected(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> subscription_;
    
    std_msgs::msg::Bool isDetected;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::Bool msg);
    
};

#endif