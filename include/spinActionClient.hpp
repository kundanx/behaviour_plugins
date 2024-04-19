#ifndef SPIN_ACTION_CLIENT_HPP
#define SPIN_ACTION_CLIENT_HPP

#include <string>
#include <memory>

#include "nav2_msgs/action/spin.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"



class spinActionClient : public BT::StatefulActionNode
{
    public:
    spinActionClient(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;
    using float32 = std_msgs::msg::Float32;


    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<Spin>::SharedPtr action_client_ptr_;

    Spin::Goal goal_spin_yaw;
    bool done_flag;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;


    // Action client result callback
    void spin_result_callback(const GoalHandleSpin::WrappedResult &result);

    // Action client feedback callback
    void spin_feedback_callback(GoalHandleSpin::SharedPtr, const std::shared_ptr<const Spin::Feedback> feedback);
    
};

#endif