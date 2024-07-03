#ifndef CV_CONFIG_HPP
#define CV_CONFIG_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"


enum VisionType
{
    EH_BALLZ,
    SILO_DETECTION
};

class Cv_Config : public BT::SyncActionNode
{
    public:
    Cv_Config(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

//    using ife_cycle

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr ball_client_ptr_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr silo_client_ptr_;

    uint8_t vision_type;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
#endif