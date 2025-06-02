#ifndef NAVIGATION_BEHAVIOUR_H
#define NAVIGATION_BEHAVIOUR_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include "behaviour_plugins/bezier.h"
#include "behaviour_plugins/angle_conversions.hpp"
#include "oakd_msgs/msg/state_pose.hpp"

#include "behaviour_plugins/robotlibpc/cpp/common/time.hpp"

#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

class GoToBallPose : public BT::StatefulActionNode
{
    public:
    GoToBallPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PosMsg = geometry_msgs::msg::PoseStamped;


    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr color_feedback_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr ball_detection_slow_down_publisher;


    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;
    std::shared_ptr<rclcpp::Subscription<oakd_msgs::msg::StatePose>> ball_track_subscription_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int8>> subscription_team_color;  
    std::shared_ptr<GoalHandleNav> goal_handle = nullptr;

    enum TeamColor
    {
        RED = -1,
        BLUE = 1
    }team_color;
    
    uint32_t start_time;
    float prev_x;
    float prev_y;

    bool done_flag;
    bool goal_sent_once;
    nav_msgs::msg::Odometry odom_msg;
    NavigateToPose::Goal goal_msg;


    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void cancel_goal();                    
    void nav_to_pose_compute_goal();
    void get_curve_points(double input_points[2][3], double output_points[5][3]);

    // Subscribers callback
    void odometry_callback(const nav_msgs::msg::Odometry &msg);
    void team_color_callback(const std_msgs::msg::Int8 &msg);

    // Action client feedbacks
    void goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_);
    void goal_result_callback(const GoalHandleNav::WrappedResult &result);
    void goal_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif