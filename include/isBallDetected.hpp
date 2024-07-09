#ifndef IS_BALL_DETECTED_H
#define IS_BALL_DETECTED_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "oakd_msgs/msg/state_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#define MAX_NOT_DETECTED 10


using namespace BT;
class isBallDetected : public BT::SyncActionNode
{
    public:
    isBallDetected(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<oakd_msgs::msg::StatePose>> subscription_;
    
    std_msgs::msg::Bool isDetected;
    oakd_msgs::msg::StatePose ball;
    uint64_t ball_not_detected_counter;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void subscriber_callback(oakd_msgs::msg::StatePose);
    
};

#endif