#include "ZoneAndColorConfig.hpp"   

isStartOrRetryZone::isStartOrRetryZone(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "robot_config", qos_profile, std::bind(&isStartOrRetryZone::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"isStartOrRetryZone::Ready..");
}
BT::PortsList isStartOrRetryZone::providedPorts()
{
    return {BT::OutputPort<uint8_t>("OP_Team_color")};
}

 BT::NodeStatus isStartOrRetryZone::tick()
 {  
    if(zone == START){
        RCLCPP_INFO(node_ptr_->get_logger(),"isStartOrRetryZone::START Zone");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"isStartOrRetryZone::RETRY Zone");
    return BT::NodeStatus::FAILURE;
 }

void isStartOrRetryZone::subscriber_callback(std_msgs::msg::Uint8 msg)
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







