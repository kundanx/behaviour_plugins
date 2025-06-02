#ifndef PHOTO_CAPTURE_HPP
#define PHOTO_CAPTURE_HPP

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/u_int8.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class StartPhotoCapture : public BT::SyncActionNode
{
    public:
    StartPhotoCapture(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> publisher_;
    
    // Methods override (uncomment if you have ports to I/O data)
    BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    
};

#endif