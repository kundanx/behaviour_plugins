#ifndef INITIALLIZE_ACTUATORS_H
#define INITIALLIZE_ACTUATORS_H

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/u_int8.hpp"

using namespace BT;


class InitiallizeActuators : public BT::SyncActionNode
{
    public:
    InitiallizeActuators(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);
    
    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};

#endif