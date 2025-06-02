#include "actions/PneumaticOn.hpp"

PneumaticOn::PneumaticOn(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
     // Mechanism is turnedOff intially
    setOutput<bool>("Op_PneumaticStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("PneumaticOn"),"PneumaticOn::Ready");
   
}

BT::PortsList PneumaticOn::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_PneumaticStatus")
};   
}
BT::NodeStatus PneumaticOn::tick() 
{
    setOutput<bool>("Op_PneumaticStatus", true);
    RCLCPP_INFO(rclcpp::get_logger("PneumaticOn"),"Pneumatic Turned On");
    return BT::NodeStatus::SUCCESS;
}