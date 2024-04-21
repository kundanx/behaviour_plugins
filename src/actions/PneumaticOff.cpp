#include "actions/PneumaticOff.hpp"

PneumaticOff::PneumaticOff(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_PneumaticStatus", false);
    std::printf("PneumaticOn action ready.. \n");

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
    std::cout<<"Pneumatic Turned Off"<<std::endl;
    return BT::NodeStatus::SUCCESS;
}