#ifndef GO_TO_SILO_HPP
#define GO_TO_SILO_HPP

#include <string>
#include <memory>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>

#include "yaml-cpp/yaml.h"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "behaviour_plugins/angle_conversions.hpp"


enum JUNCTION_TYPE
{
    JUNCTION_NOT_DETECTED,
    LEFT_TURN,
    RIGHT_TURN,
    CROSS_JUNCTION,
    T_JUNCTION,
    RIGHT_T_JUNCTION,
    LEFT_T_JUNCTION,
    X_HORIZONTAL_LINE,
    Y_VERTICAL_LINE,
    L_JUNCTION,
    MIRROR_L_JUNCTIO 

};
enum ROBOT_POSE_WRT_SILO
{
    LEFT,
    RIGHT,
    CENTRE
};


class GoToSiloPose : public BT::StatefulActionNode
{
    public:
    GoToSiloPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using PosMsg = geometry_msgs::msg::PoseStamped;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;


    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::TimerBase::SharedPtr goal_abort_timer;

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr color_feedback_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr go_to_silo_publisher;

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscription_odometry;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>> subscription_silonumber;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_junctiontype;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> subscription_isOnLine;  
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int8>> subscription_team_color; 
    std::shared_ptr<GoalHandleNav> goal_handle;

    enum TeamColor
    {
        RED = -1,
        BLUE = 1
    }team_color;

    bool done_flag;
    bool is_on_line;
    bool x_horiz_line_detected;
    double y_coordinate;
    std::vector<float> pose{2};

    NavigateToPose::Goal goal_msg;
    nav_msgs::msg::Odometry odom_msg;
    std_msgs::msg::UInt8MultiArray silo_numbers;

    bool testing = false;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Compute goal poses
    void compute_goal_poses();
    int compute_goal_NavTo();
    void cancel_goal();

    // Subscriber callback
    void silo_subscriber_callback(const std_msgs::msg::UInt8MultiArray &msg);
    void junction_subscriber_callback(const std_msgs::msg::UInt8 &msg);
    void line_subscriber_callback(const std_msgs::msg::Bool &msg);
    void odometry_callback(const nav_msgs::msg::Odometry &msg);
    void team_color_callback(const std_msgs::msg::Int8 &msg);

    //Action server feedbacks
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_);
    void result_callback(const GoalHandleNav::WrappedResult &result);
    void feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif