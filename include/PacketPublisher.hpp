#ifndef PACKET_PUBLISHER_H
#define PACKET_PUBLISHER_H

#include <string>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/int8_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

/*
    Struct to define Serial data packet for actuators states
*/
// struct data
// {
//     uint8_t penumaticOn = 0x00000001;
//     uint8_t penumaticOff = 0x0000000;
//     uint8_t rollerOn = 0x0000010;
//     uint8_t rollerOn = 0x000000;
    
// }

class PacketPublisher : public BT::SyncActionNode
{
    // private:
    // bool this->last_RollerStatus, this->last_ConveyerStatus;
    // uint8_t this->last_RollerSpeed, this->last_conveyerSpeed;
    // bool this->last_PneumaticStatus;

    public:
    PacketPublisher(
        const std::string &name,
        const BT::NodeConfiguration &config,
        rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8MultiArray>> publisher_;
    
    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // Publisher callback
    void publsiher_callback();
    
};

#endif