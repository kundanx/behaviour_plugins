#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "yaml-cpp/yaml.h"


class GoToPose : public BT::StatefulActionNode
{
    public:
    GoToPose(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;

    double xyz[3]; // x, y, z
    double q[4];   // x, y, z , w
    bool done_flag;

    // // Methods override (uncomment if you have ports to I/O data)
    // static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    // Subscriber callback
    void ball_pose_callback(const geometry_msgs::msg::Pose & msg);

    // Action client callback
    void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
    
};