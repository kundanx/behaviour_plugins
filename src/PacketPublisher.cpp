#include "PacketPublisher.hpp"   

PacketPublisher::PacketPublisher(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::Int8MultiArray>( "/act_vel", 10);
    RCLCPP_INFO(node_ptr_->get_logger(),"PacketPublisher node Ready..");
}

BT::PortsList PacketPublisher::providedPorts()
{
    return {
        BT::InputPort<bool>("Ip_RollerStatus"),
        BT::InputPort<int>("Ip_RollerSpeed"),   
        BT::InputPort<bool>("Ip_ConveyerStatus"),
        BT::InputPort<int>("Ip_ConveyerSpeed"), 
        BT::InputPort<bool>("Ip_PneumaticStatus")
    };
}

 BT::NodeStatus PacketPublisher::tick()
 {  
    auto ConveyerStatus = getInput<bool>("Ip_ConveyerStatus");
    auto ConveyerSpeed = getInput<int>("Ip_ConveyerSpeed");
    auto RollerStatus = getInput<bool>("Ip_RollerStatus");
    auto RollerSpeed = getInput<int>("Ip_RollerSpeed");
    auto PneumaticStatus = getInput<bool>("Ip_PneumaticStatus");

    std_msgs::msg::Int8MultiArray msg;
    
    msg.data[0] = RollerSpeed.value();
    msg.data[1] = RollerStatus.value();
    msg.data[2] = ConveyerSpeed.value();
    msg.data[3] =  ConveyerStatus.value();
    msg.data[4] = PneumaticStatus.value();

    // publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
 }

















