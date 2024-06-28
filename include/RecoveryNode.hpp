#ifndef RECOVERY_NODE_H
#define RECOVERY_NODE_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>


#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"


#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

enum RecoveryState
{
    NAV,
    BACKUP,
    SPIN,
    HALT
};

class RecoveryNode : public BT::StatefulActionNode
{
    public:
    RecoveryNode(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using BackUp = nav2_msgs::action::BackUp;
    using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;

    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;


    using PosMsg = geometry_msgs::msg::PoseStamped;
    using float64 = std_msgs::msg::Float64;
    using float32 = std_msgs::msg::Float32;


    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_ptr_;
    rclcpp_action::Client<BackUp>::SharedPtr backUp_action_client_ptr_;
    rclcpp_action::Client<Spin>::SharedPtr spin_action_client_ptr_;


    std::shared_ptr<GoalHandleNav> nav_goal_handle;
    std::shared_ptr<GoalHandleBackUp> backUp_goal_handle;
    std::shared_ptr<GoalHandleSpin> spin_goal_handle;




    double xyz[3]; // x, y, z
    double q[4];   // x, y, z , w
    bool done_flag;
    nav_msgs::msg::Odometry odom_msg;

    // Methods override (uncomment if you have ports to I/O data)
    // static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Odometry callback
    void odometry_callback(const nav_msgs::msg::Odometry &msg);


    void nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_);
    void nav_result_callback(const GoalHandleNav::WrappedResult &result);
    void nav_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void backUp_goal_response_callback(const rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr & goal_handle_);
    void backUp_result_callback(const GoalHandleBackUp::WrappedResult &result);
    void backUp_feedback_callback(GoalHandleBackUp::SharedPtr, const std::shared_ptr<const BackUp::Feedback> feedback);

    void spin_goal_response_callback(const rclcpp_action::ClientGoalHandle<Spin>::SharedPtr & goal_handle_);
    void spin_result_callback(const GoalHandleSpin::WrappedResult &result);
    void spin_feedback_callback(GoalHandleSpin::SharedPtr, const std::shared_ptr<const Spin::Feedback> feedback);
    
};

#endif