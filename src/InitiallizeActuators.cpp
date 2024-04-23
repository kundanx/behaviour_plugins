#include "InitiallizeActuators.hpp"   

InitiallizeActuators::InitiallizeActuators(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators node Ready..");
}

BT::PortsList InitiallizeActuators::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_RollerStatus"),
        BT::OutputPort<int>("Op_RollerSpeed"),   
        BT::OutputPort<bool>("Op_ConveyerStatus"),
        BT::OutputPort<int>("Op_ConveyerSpeed"), 
        BT::OutputPort<bool>("Op_PneumaticStatus")
    };
}

 BT::NodeStatus InitiallizeActuators::tick()
 {  
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);
    setOutput<bool>("Op_RollerStatus", false);
    setOutput<int>("Op_RollerSpeed", 1);
    setOutput<bool>("Op_PneumaticStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Actuators in reset state..");
    return BT::NodeStatus::SUCCESS;
 }

















