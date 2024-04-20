#include "actions/TurnOffConveyer.hpp"

TurnOffConveyer::TurnOffConveyer(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_ConveyerStatus", false);
}

BT::PortsList TurnOffConveyer::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_ConveyerStatus")
};   
}
BT::NodeStatus TurnOffConveyer::tick() 
{
    setOutput<bool>("Op_ConveyerStatus", false);
    std::cout<<"Conveyer Turned Off"<<std::endl;
    return BT::NodeStatus::SUCCESS;
}