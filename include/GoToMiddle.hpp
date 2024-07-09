#ifndef GO_TO_MIDDLE_H
#define GO_TO_MIDDLE_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviour_plugins/angle_conversions.hpp"


enum MiddleType
{
    MIDDLE_OF_SLOPE,
    ABSOLUTE_MIDDLE
};
class GoToMiddle : public BT::StatefulActionNode
{
    public:
    GoToMiddle(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PosMsg = geometry_msgs::msg::PoseStamped;

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<GoalHandleNav> goal_handle;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int8>> subscription_team_color;  


    enum TeamColor
    {
        RED = -1,
        BLUE = 1
    }team_color;
    bool done_flag;
    nav_msgs::msg::Odometry odom_msg;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void cancel_goal();

    // Subscribers callbacks
    void odometry_callback(const nav_msgs::msg::Odometry &msg);
    void team_color_callback(const std_msgs::msg::Int8 &msg);

    // Action Server feedbacks
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_);
    void goal_result_callback(const GoalHandleNav::WrappedResult &result);
    void goal_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif