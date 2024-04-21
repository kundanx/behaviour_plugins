#include "actions/TurnOffRoller.hpp"

TurnOffRoller::TurnOffRoller(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_RollerStatus", false);
    std::cout<<"Roller Off action ready.."<<std::endl;

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
    std::cout<<"Roller Turned Off"<<std::endl;
    return BT::NodeStatus::SUCCESS;
}