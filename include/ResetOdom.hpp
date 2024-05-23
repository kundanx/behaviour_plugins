#ifndef RESET_ODOM_HPP
#define RESET_ODOM_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

// #include "geometry_msgs/msg/pose.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
enum LANDMARKS
{
    ORIGIN,
    SILO_1,
    SILO_2,
    SILO_3,
    SILO_4,
    SILO_5,
    SILO_1_Y_LINE,
    SILO_2_Y_LINE,
    SILO_3_Y_LINE,
    SILO_4_Y_LINE,
    SILO_5_Y_LINE,
    X_HORIZ_LINE
};

class ResetOdom : public BT::SyncActionNode
{
    public:
    ResetOdom(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> publisher_;
    

    // // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Subscriber callback
    
};

#endif