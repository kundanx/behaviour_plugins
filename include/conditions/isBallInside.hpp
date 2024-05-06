#ifndef IS_BALL_INSIDE_H
#define IS_BALL_INSIDE_H

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"


using namespace BT;
class isBallInside : public BT::SyncActionNode
{
    public:
    isBallInside(
        const std::string &name,
        const BT::NodeConfiguration &config);

    bool inside;
    
    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    
};

#endif