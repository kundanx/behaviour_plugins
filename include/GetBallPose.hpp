#ifndef GetBallPose_H
#define GetBallPose_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
class GetBallPose : public BT::SyncActionNode
{
    public:
    GetBallPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> subscription_;
    
    geometry_msgs::msg::PoseStamped ballPose;
    bool pose_recieved_flag;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    void ball_pose_callback(geometry_msgs::msg::PoseStamped msg);
    
};

#endif