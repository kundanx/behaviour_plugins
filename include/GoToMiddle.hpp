#ifndef GO_TO_MIDDLE_H
#define GO_TO_MIDDLE_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

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
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;

    std::shared_ptr<GoalHandleNav> goal_handle;


    bool done_flag;
    nav_msgs::msg::Odometry odom_msg;


    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Subscriber callback
    // void ball_pose_callback(const geometry_msgs::msg::Pose & msg);

    void cancel_goal();

    void odometry_callback(const nav_msgs::msg::Odometry &msg);

    // Action client goal response callback
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_);

    // Action client result callback
    void nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result);

    // // Action client feedback callback
    void nav_to_pose_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif