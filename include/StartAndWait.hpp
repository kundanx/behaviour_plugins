#ifndef START_AND_WAIT_HPP
#define START_AND_WAIT_HPP

#include <string>
#include <memory>

#include "std_msgs/msg/u_int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
class StartAndWait : public BT::SyncActionNode
{
    public:
    StartAndWait(
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