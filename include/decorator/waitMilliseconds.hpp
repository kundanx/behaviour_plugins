#ifndef WAIT_MILLI_SECONDS
#define WAIT_MILLI_SECONDS

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"



class waitMillisecond : public BT::StatefulActionNode
{
    public:
    waitMillisecond(const std::string &name,
             const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

   
    rclcpp::Node::SharedPtr node_ptr_;

    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    uint64_t start_time;
    uint64_t wait_time;

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;


};

#endif