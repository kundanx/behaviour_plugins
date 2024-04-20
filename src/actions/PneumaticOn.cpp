#include "actions/PneumaticOn.hpp"

PneumaticOn::PneumaticOn(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
     // Mechanism is turnedOff intially
    setOutput<bool>("Op_PneumaticStatus", false);
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
    std::cout<<"Pneumatic Turned On"<<std::endl;
    return BT::NodeStatus::SUCCESS;
}