#ifndef GO_TO_SILO_HPP
#define GO_TO_SILO_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "yaml-cpp/yaml.h"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

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

class GoToSiloPose : public BT::StatefulActionNode
{
    public:
    GoToSiloPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PosMsg = geometry_msgs::msg::PoseStamped;

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_silonumber;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_junctiontype;
    std::shared_ptr<GoalHandleNav> goal_handle;

    double y_coordinate;
    bool done_flag;
    uint8_t silo_number;
    bool x_horiz_line_detected;
    std::vector<float> pose;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Subscriber callback
    void silo_subscriber_callback(std_msgs::msg::UInt8);
    void junction_subscriber_callback(std_msgs::msg::UInt8);

    // cancel naviagtion goal
    void cancel_goal();

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle);

    // Action client result callback
    void result_callback(const GoalHandleNav::WrappedResult &result);

    // // Action client feedback callback
    void feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif