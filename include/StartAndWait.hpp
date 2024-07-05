#ifndef START_AND_WAIT_HPP
#define START_AND_WAIT_HPP

#include <string>
#include <memory>

#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

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

    
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PosMsg = geometry_msgs::msg::PoseStamped;

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

    
    bool start;
    bool wait;

  
    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8 msg);
    
};

#endif