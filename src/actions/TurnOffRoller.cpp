#include "actions/TurnOffRoller.hpp"

TurnOffRoller::TurnOffRoller(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_RollerStatus", false);
    setOutput<int>("Op_RollerSpeed", 1);
    
    RCLCPP_INFO(rclcpp::get_logger("TurnOffRoller"),"TurnOffRoller::Ready");
}

BT::PortsList TurnOffRoller::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_RollerStatus"),
        BT::OutputPort<int>("Op_RollerSpeed")
    };   
}
BT::NodeStatus TurnOffRoller::tick() 
{
    setOutput<bool>("Op_RollerStatus", false);
    setOutput<int>("Op_RollerSpeed", 1);

    RCLCPP_INFO(rclcpp::get_logger("TurnOffRoller"),"Roller Turned Off..");
    return BT::NodeStatus::SUCCESS;
}