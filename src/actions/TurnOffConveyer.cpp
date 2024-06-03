#include "actions/TurnOffConveyer.hpp"

TurnOffConveyer::TurnOffConveyer(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    // Mechanism is turnedOff intially
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);


    RCLCPP_INFO(rclcpp::get_logger("TurnOffConveyer"),"TurnOffConveyer action Ready..");
}

BT::PortsList TurnOffConveyer::providedPorts()
{
return {
    BT::OutputPort<bool>("Op_ConveyerStatus"),
    BT::OutputPort<int>("Op_ConveyerSpeed")
};   
}
BT::NodeStatus TurnOffConveyer::tick() 
{
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);
    RCLCPP_INFO(rclcpp::get_logger("TurnOffConveyer"),"Conveyer Turned Off..");
    return BT::NodeStatus::SUCCESS;
}