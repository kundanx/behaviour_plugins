#include "actions/TurnOnConveyer.hpp"

TurnOnConveyer::TurnOnConveyer(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
     // Mechanism is turnedOff intially
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);
    RCLCPP_INFO(rclcpp::get_logger("TurnOnConveyer"),"TurnOnConveyer::Ready");
}

BT::PortsList TurnOnConveyer::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_ConveyerStatus"),
        BT::OutputPort<int>("Op_ConveyerSpeed"),
        BT::InputPort<int>("Ip_ConveyerSpeed")
    };   
}
BT::NodeStatus TurnOnConveyer::tick() 
{
    auto speed = getInput<int>("Ip_ConveyerSpeed");
    if(!speed)
    { 
        RCLCPP_INFO(rclcpp::get_logger("TurnOnConveyer"),"Input Port Error: [In_ConveyerSpeed]");
        return BT::NodeStatus::FAILURE;
    }
    setOutput<bool>("Op_ConveyerStatus", true);
    setOutput<int>("Op_ConveyerSpeed", speed.value());
    RCLCPP_INFO(rclcpp::get_logger("TurnOnConveyer"),"Conveyer Turned Speed[%i]",speed.value());
    return BT::NodeStatus::SUCCESS;
}