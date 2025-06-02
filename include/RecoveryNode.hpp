#ifndef RECOVERY_NODE_H
#define RECOVERY_NODE_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include "behaviour_plugins/angle_conversions.hpp"

#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "action_pkg/action/line_follow.hpp"

#include "behaviour_plugins/robotlibpc/cpp/common/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "oakd_msgs/msg/state_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#define MAX_SPIN_NUM 3
#define MAX_BACKUP_NUM 3
#define MAX_ALIGN_YAW_NUM 3
enum RecoveryState
{
    NAV,
    BACKUP,
    HALT
};
enum LeftOrRight
{
    LEFT,
    RIGHT,
    MIDDLE,
    N0_CHANGE
};

class RecoveryNode : public BT::StatefulActionNode
{
    public:
    RecoveryNode(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using LineFollow = action_pkg::action::LineFollow;
    using GoalHandleLineFollow = rclcpp_action::ClientGoalHandle<LineFollow>;

    // using PosMsg = geometry_msgs::msg::PoseStamped;
    // using float64 = std_msgs::msg::Float64;
    // using float32 = std_msgs::msg::Float32;


    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int8>> subscription_team_color;  


    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr color_feedback_publisher;


    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_ptr_;
    rclcpp_action::Client<LineFollow>::SharedPtr backUp_action_client_ptr_;



    std::shared_ptr<GoalHandleNav> nav_goal_handle;
    std::shared_ptr<GoalHandleLineFollow> backUp_goal_handle;

    enum TeamColor
    {
        RED = -1,
        BLUE = 1
    }team_color;


    // uint8_t align_yaw_counter;
    
    // double previous_ball_theta;
    bool done_flag;
    bool in_middle;

    uint32_t start_time;
    float prev_x;
    float prev_y;
    LeftOrRight left_right;
    RecoveryState recovery_state;
    
    nav_msgs::msg::Odometry odom_msg;
    oakd_msgs::msg::StatePose ball_pose;

    // Methods override (uncomment if you have ports to I/O data)
    // static BT::PortsList providedPorts();

    void cancel_goal();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Odometry callback
    void odometry_callback(const nav_msgs::msg::Odometry &msg);
    void team_color_callback(const std_msgs::msg::Int8 &msg);


    void nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_);
    void nav_result_callback(const GoalHandleNav::WrappedResult &result);
    void nav_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void backUp_goal_response_callback(const rclcpp_action::ClientGoalHandle<LineFollow>::SharedPtr & goal_handle_);
    void backUp_result_callback(const GoalHandleLineFollow::WrappedResult &result);
    void backUp_feedback_callback(GoalHandleLineFollow::SharedPtr, const std::shared_ptr<const LineFollow::Feedback> feedback);

};

#endif