#include "actions/TurnOnRoller.hpp"

TurnOnRoller::TurnOnRoller(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_RollerStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("TurnOnRoller"),"TurnOnRoller action Ready..");
}

BT::PortsList TurnOnRoller::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_RollerStatus"),
    BT::OutputPort<int>("Op_RollerSpeed"),
    BT::InputPort<int>("Ip_RollerSpeed")
};   
}
BT::NodeStatus TurnOnRoller::tick() 
{
    auto speed = getInput<int>("Ip_RollerSpeed");
    if(!speed)
    { 
        RCLCPP_INFO(rclcpp::get_logger("TurnOnRoller"),"Input Port Error: [In_RollerSpeed]");
        return BT::NodeStatus::FAILURE;
    }
    setOutput<bool>("Op_RollerStatus", true);
    setOutput<int>("Op_RollerSpeed", speed.value());
    RCLCPP_INFO(rclcpp::get_logger("TurnOnRoller"),"Roller Turned On: Speed[%i]",speed.value());
    return BT::NodeStatus::SUCCESS;
}