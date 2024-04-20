#include "actions/TurnOnConveyer.hpp"

TurnOnConveyer::TurnOnConveyer(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
     // Mechanism is turnedOff intially
    setOutput<bool>("Op_ConveyerStatus", false);
}

BT::PortsList TurnOnConveyer::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_ConveyerStatus"),
        BT::OutputPort<uint8_t>("Op_ConveyerSpeed"),
        BT::InputPort<uint8_t>("Ip_ConveyerSpeed")
    };   
}
BT::NodeStatus TurnOnConveyer::tick() 
{
    auto speed = getInput<uint8_t>("Ip_ConveyerSpeed");
    if(!speed)
    { 
        std::cout<<"Input Port Error: [In_ConveyerSpeed]"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }
    setOutput<bool>("Op_ConveyerStatus", true);
    setOutput<uint8_t>("Op_ConveyerSpeed", speed.value());
    std::cout<<"Conveyer Turned On: Speed["<<speed.value()<<"]"<<std::endl;

    return BT::NodeStatus::SUCCESS;
}