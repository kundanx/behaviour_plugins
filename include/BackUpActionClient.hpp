#ifndef BACK_UP_ACTION_CLIENT_HPP
#define BACK_UP_ACTION_CLIENT_HPP

#include <string>
#include <memory>

#include "nav2_msgs/action/back_up.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"



class BackUpActionClient : public BT::StatefulActionNode
{
    public:
    BackUpActionClient(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    using BackUp = nav2_msgs::action::BackUp;
    using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;
    using float64 = std_msgs::msg::Float64;
    using float32 = std_msgs::msg::Float32;



    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<BackUp>::SharedPtr action_client_ptr_;

    BackUp::Goal backUp_dist;
    bool done_flag;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;


    // Action client result callback
    void backUp_result_callback(const GoalHandleBackUp::WrappedResult &result);

    // Action client feedback callback
    void backUp_feedback_callback(GoalHandleBackUp::SharedPtr, const std::shared_ptr<const BackUp::Feedback> feedback);
    
};

#endif