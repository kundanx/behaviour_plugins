#ifndef NAVIGATION_BEHAVIOUR_H
#define NAVIGATION_BEHAVIOUR_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include "behaviour_plugins/bezier.h"
#include "behaviour_plugins/angle_conversions.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"



// Quaternion ToQuaternion(double roll, double pitch, double yaw); // roll (x), pitch (y), yaw (z), angles are in radians
// EulerAngles ToEulerAngles(double w, double x, double y, double z) ;

class GoToBallPose : public BT::StatefulActionNode
{
    public:
    GoToBallPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    using PosMsg = geometry_msgs::msg::PoseStamped;

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_ptr_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_;
    std::shared_ptr<GoalHandleNav> goal_handle;
   


    double xyz[3]; // x, y, z
    double q[4];   // x, y, z , w
    bool done_flag;
    nav_msgs::msg::Odometry curr_pose;
    NavigateThroughPoses::Goal goal_msg;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void compute_goal();
    void get_curve_points(double input_points[2][3], double output_points[5][3]);

    // Subscriber callback
    // void ball_pose_callback(const geometry_msgs::msg::Pose & msg);

    // Odometry callback
    void odometry_callback(const nav_msgs::msg::Odometry &msg);

    // Action client goal response
    void goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_);

    // Action client result callback
    void goal_result_callback(const GoalHandleNav::WrappedResult &result);

    // // Action client feedback callback
    void goal_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    
};

#endif