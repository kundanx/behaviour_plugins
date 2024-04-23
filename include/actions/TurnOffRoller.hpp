#ifndef TURN_OFF_ROLLER_HPP
#define TURN_OFF_ROLLER_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"


using namespace BT;
class TurnOffRoller : public BT::SyncActionNode
{
    public:
    TurnOffRoller(
        const std::string &name,
        const BT::NodeConfiguration &config
    );
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};

#endif