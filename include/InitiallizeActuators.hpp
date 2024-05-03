#ifndef INITIALLIZE_ACTUATORS_H
#define INITIALLIZE_ACTUATORS_H

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;


class InitiallizeActuators : public BT::SyncActionNode
{
    public:
    InitiallizeActuators(
        const std::string &name,
        const BT::NodeConfiguration &config);
    
    // Methods override (uncomment if you have ports to I/O data)
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    
};

#endif