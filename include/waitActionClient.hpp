#ifndef WAIT_ACTION_CLIENT_HPP
#define WAIT_ACTION_CLIENT_HPP

#include <string>
#include <memory>

#include "nav2_msgs/action/wait.hpp"
#include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/uint32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"


class waitActionClient : public BT::StatefulActionNode
{
    public:
    waitActionClient(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    using Wait = nav2_msgs::action::Wait;
    using GoalHandleWait = rclcpp_action::ClientGoalHandle<Wait>;
    using int32 = std_msgs::msg::Int32;


    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<Wait>::SharedPtr action_client_ptr_;
    std::shared_ptr<GoalHandleWait> goal_handle;


    Wait::Goal wait;
    bool done_flag;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Action client goal response callback
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr & goal_handle_);

    // Action client result callback
    void wait_result_callback(const GoalHandleWait::WrappedResult &result);

    // Action client feedback callback
    void wait_feedback_callback(GoalHandleWait::SharedPtr, const std::shared_ptr<const Wait::Feedback> feedback);
    
};

#endif