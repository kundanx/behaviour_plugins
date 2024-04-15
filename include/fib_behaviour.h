#ifndef FIB_BEHAVIOUR_H
#define FIB_BEHAVIOUR_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "action_pkg/action/action.hpp"

#include "yaml-cpp/yaml.h"
#include <string>

class Fib : public BT::StatefulActionNode
{
    public:
    Fib(const std::string &name,
                        const BT::NodeConfiguration &config,
                        rclcpp::Node::SharedPtr node_ptr);
    using Fibonacci = action_pkg::action::Action;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    rclcpp::Node::SharedPtr node_ptr__;
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_ptr_;
    bool done_flag;

    //methods override
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    //Action client callbacks
    void feedback_callback(GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback);
    void result_callback(const GoalHandleFibonacci::WrappedResult &result);

};

#endif