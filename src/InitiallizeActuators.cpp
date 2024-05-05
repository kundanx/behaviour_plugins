#include "InitiallizeActuators.hpp"   

InitiallizeActuators::InitiallizeActuators(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "/ball_roller_status", 10, std::bind(&InitiallizeActuators::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators node Ready..");
}

BT::PortsList InitiallizeActuators::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_RollerStatus"),
        BT::OutputPort<int>("Op_RollerSpeed"),   
        BT::OutputPort<bool>("Op_ConveyerStatus"),
        BT::OutputPort<int>("Op_ConveyerSpeed"), 
        BT::OutputPort<bool>("Op_PneumaticStatus"),
        BT::OutputPort<bool>("OP_IsBallInside"),
        BT::OutputPort<bool>("OP_IsOnlyBall")
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

 void InitiallizeActuators::subscriber_callback(std_msgs::msg::UInt8 msg)
 {
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators subs callback..");
    switch (msg.data)
    {
        case 1: 
            RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Red Ball detected");
            setOutput<bool>("OP_IsBallInside", true);
            break;
        case 2:
            RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Blue Ball detected");
            setOutput<bool>("OP_IsBallInside", false);
            break;
        case 3:
            RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"purple Ball detected");
            setOutput<bool>("OP_IsBallInside", false);
            break;

        default:
            RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"No ball detected");
            setOutput<bool>("OP_IsBallInside", false);
    }
 }

















