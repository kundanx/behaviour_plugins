#include "actions/TurnOnRoller.hpp"

TurnOnRoller::TurnOnRoller(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_RollerStatus", false);
    std::cout<<"Roller On action ready.."<<std::endl;

}

BT::PortsList TurnOnRoller::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_RollerStatus"),
    BT::OutputPort<uint8_t>("Op_RollerSpeed"),
    BT::InputPort<uint8_t>("Ip_RollerSpeed")
};   
}
BT::NodeStatus TurnOnRoller::tick() 
{
    auto speed = getInput<uint8_t>("Ip_RollerSpeed");
    if(!speed)
    { 
        std::cout<<"Input Port Error: [In_RollerSpeed] \n";
        return BT::NodeStatus::FAILURE;
    }
    setOutput<bool>("Op_RollerStatus", true);
    setOutput<uint8_t>("Op_RollerSpeed", speed.value());
    std::cout<<"Roller Turned On: Speed["<<speed.value()<<"]"<<std::endl;
    return BT::NodeStatus::SUCCESS;
}