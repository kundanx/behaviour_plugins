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
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_;


    double xyz[3]; // x, y, z
    double q[4];   // x, y, z , w
    bool done_flag;
    bool goal_sent_flag;
    uint8_t silo_number;
    std::vector<float> pose;

    // Methods override (uncomment if you have ports to I/O data)
    // static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Subscriber callback
    void subscriber_callback(std_msgs::msg::UInt8);

    // Action client result callback
    void nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result);

    // // Action client feedback callback
    void nav_to_pose_feedback_callback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    
};

#endif