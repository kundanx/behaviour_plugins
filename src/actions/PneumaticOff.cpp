#include "actions/PneumaticOff.hpp"

PneumaticOff::PneumaticOff(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_PneumaticStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("PneumaticOff"),"PneumaticOff::Ready..");
    // std::cout<<"PneumaticOn  ready.. "<<std::endl;

}

BT::PortsList PneumaticOff::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_PneumaticStatus")
};   
}
BT::NodeStatus PneumaticOff::tick() 
{
    setOutput<bool>("Op_PneumaticStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("PneumaticOff"),"Pneumatic Turned Off");
    return BT::NodeStatus::SUCCESS;
}