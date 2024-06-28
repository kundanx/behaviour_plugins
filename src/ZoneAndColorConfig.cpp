#include "ZoneAndColorConfig.hpp"   

ZoneAndColorConfig::ZoneAndColorConfig(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "robot_config", qos_profile, std::bind(&ZoneAndColorConfig::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::Ready..");
}
BT::PortsList ZoneAndColorConfig::providedPorts()
{
    return {BT::OutputPort<uint8_t>("OP_Team_color")};
}

 BT::NodeStatus ZoneAndColorConfig::tick()
 {  
    if(zone == START){
        RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::START Zone");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::RETRY Zone");
    return BT::NodeStatus::FAILURE;
 }

void ZoneAndColorConfig::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data & 0x01)
        zone = START;    
    else if( msg.data & 0x02)
        zone = RETRY;
    
    if(msg.data & 0x04)
        team_color = BLUE;
    else if( msg.data & 0x08)
        team_color = RED;
    
    setOutput<uint8_t>("OP_Team_color", team_color);
}







