#ifndef IS_ONLY_BALL_H
#define IS_ONLY_BALL_H

#include <string>
#include <memory>

#include "std_msgs/msg/u_int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
class isOnlyBall : public BT::SyncActionNode
{
    public:
    isOnlyBall(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool onlyBall;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};

#endif