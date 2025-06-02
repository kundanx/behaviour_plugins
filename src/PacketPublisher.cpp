#include "PacketPublisher.hpp"   

PacketPublisher::PacketPublisher(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>( "act_vel", 10);
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

    auto ConveyerStatus_ = getInput<bool>("Ip_ConveyerStatus");
    auto ConveyerSpeed_ = getInput<int>("Ip_ConveyerSpeed");
    auto RollerStatus_ = getInput<bool>("Ip_RollerStatus");
    auto RollerSpeed_ = getInput<int>("Ip_RollerSpeed");
    auto PneumaticStatus_ = getInput<bool>("Ip_PneumaticStatus");

  
    if(!RollerSpeed_)
        throw BT::RuntimeError("[Packet Publisher] error reading RollerSpeed_");
    if(!ConveyerSpeed_)
        throw BT::RuntimeError("[Packet Publisher] error reading ConveyerSpeed_");
    if(!RollerStatus_)
        throw BT::RuntimeError("[Packet Publisher] error reading RollerStatus_");
    if(!ConveyerStatus_)
        throw BT::RuntimeError("[Packet Publisher] error reading ConveyerStatus_");
    if(!PneumaticStatus_)
        throw BT::RuntimeError("[Packet Publisher] error reading PneumaticStatus_");
    


    auto ConveyerStatus = ConveyerStatus_.value();
    auto ConveyerSpeed = ConveyerSpeed_.value();
    auto RollerStatus = RollerStatus_.value();
    auto RollerSpeed = RollerSpeed_.value();
    auto PneumaticStatus = PneumaticStatus_.value();
    std_msgs::msg::UInt8MultiArray msg;
    // msg.layout.dim =  (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*3)
    // msg.data = (uint8_t *)malloc(sizeof(uint8_t)*3);
    msg.data.resize(3);
    msg.data[0]= RollerSpeed;
    msg.data[1]= ConveyerSpeed;
    msg.data[2]=0x00;

    if (RollerStatus)
        msg.data[2] = msg.data[2] | 0b00000001 ;
    
    if (ConveyerStatus)
        msg.data[2] = msg.data[2] | 0b00000010;
    
    if (PneumaticStatus)
        msg.data[2] = msg.data[2] | 0b00000100;

    RCLCPP_INFO(rclcpp::get_logger("PacketPublisher"),"Roller:%i,conveyer: %i, pneumatic: %i", (int)RollerStatus, (int)ConveyerStatus, (int)PneumaticStatus);

    publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
 }

















