#include "actions/TurnOffRoller.hpp"

TurnOffRoller::TurnOffRoller(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_RollerStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("TurnOffRoller"),"TurnOffRoller action Ready..");
}

BT::PortsList TurnOffRoller::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_RollerStatus")
};   
}
BT::NodeStatus TurnOffRoller::tick() 
{
    setOutput<bool>("Op_RollerStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("TurnOffRoller"),"Roller Turned Off..");
    return BT::NodeStatus::SUCCESS;
}