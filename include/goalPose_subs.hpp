#ifndef GOALPOSE_SUBS_H
#define GOALPOSE_SUBS_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "behaviortree_cpp/bt_factory.h"
#include "yaml-cpp/yaml.h"

using namespace BT;
class GoalPoseSubs : public BT::SyncActionNode
{
    public:
    GoalPoseSubs(const std::string &name,
             rclcpp::Node::SharedPtr node_ptr);

    using PosMsg = geometry_msgs::msg::PoseStamped;

    rclcpp::Node::SharedPtr node_ptr_;
    // rclcpp_action::Client<PosMsg>::SharedPtr action_client_ptr_;

    double xyz[]={0.0, 0.0, 0.0}; // x, y, z
    double q[] = {0.0, 0.0, 0.0, 1.0};   // x, y, z , w
    auto poseStamped = geometry_msgs::msg::PoseStamped();
    bool done_flag;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void ball_pose_callback(const geometry_msgs::msg::Pose & msg);
    
};

#endif