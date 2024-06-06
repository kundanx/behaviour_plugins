#ifndef LINE_FOLLOW_HPP
#define LINE_FOLLOW_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_pkg/action/line_follow.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
enum ActionType
{
    NAVIGATE_FROM_START_ZONE,
    NAVIGATE_FROM_RETRY_ZONE,
    ALIGN_W_SILO,
    ALIGN_YAW
};

class LineFollower : public BT::StatefulActionNode  
{
    public:
    using LineFollow = action_pkg::action::LineFollow;
    using GoalHandleLineFollow = rclcpp_action::ClientGoalHandle<LineFollow>;


    LineFollower(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    bool done_flag;

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr area3_reached_publisher;
    rclcpp_action::Client<LineFollow>::SharedPtr action_client_ptr_;
    std::shared_ptr<GoalHandleLineFollow> goal_handle;
    
    
    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();


    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    void cancel_goal();
    void goal_response_callback(const GoalHandleLineFollow::SharedPtr & goal_handle);
    void feedback_callback( GoalHandleLineFollow::SharedPtr, const std::shared_ptr<const LineFollow::Feedback>);
    void result_callback(const GoalHandleLineFollow::WrappedResult & result);
    
};

#endif