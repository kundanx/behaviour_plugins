#ifndef RETURN_SUCCESS_NODE_HPP
#define RETURN_SUCCESS_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"


using namespace BT;
class returnSuccess : public BT::SyncActionNode
{
    public:
    returnSuccess(
        const std::string &name,
        const BT::NodeConfiguration &config
    );
    // static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
#endif