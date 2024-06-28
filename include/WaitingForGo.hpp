#ifndef WAITING_FOR_GO_HPP
#define WAITING_FOR_GO_HPP

#include <string>
#include <memory>

#include "std_msgs/msg/u_int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
class WaitingForGo : public BT::SyncActionNode
{
    public:
    WaitingForGo(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    
    bool start;
    bool wait;

  
    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};

#endif