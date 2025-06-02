#include "ZoneAndColorConfig.hpp"   

ZoneAndColorConfig::ZoneAndColorConfig(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    color_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int8>( "team_color", 
        qos_profile, 
        std::bind(&ZoneAndColorConfig::color_subscriber_callback,
        this,
        std::placeholders::_1));
    zone_subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "zone_status", 
        qos_profile, 
        std::bind(&ZoneAndColorConfig::zone_subscriber_callback,
        this,
        std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::Ready..");
}
BT::PortsList ZoneAndColorConfig::providedPorts()
{
    return {BT::OutputPort<int>("OP_Team_color")};
}

 BT::NodeStatus ZoneAndColorConfig::tick()
 {  
    // if ( team_color == RED)
    //     RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::RED Team");
    // else
    //     RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::BLUE Team");

    if(zone == START){
        // RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::START Zone");
        return BT::NodeStatus::SUCCESS;
    }
    // RCLCPP_INFO(node_ptr_->get_logger(),"ZoneAndColorConfig::RETRY Zone");
    return BT::NodeStatus::FAILURE;
 }

void ZoneAndColorConfig::color_subscriber_callback(std_msgs::msg::Int8 msg)
{   

    if(msg.data == 1)
        team_color = BLUE;
    else 
        team_color = RED;
    
    setOutput<int>("OP_Team_color", team_color);
}

void ZoneAndColorConfig::zone_subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1)
        zone = START;    
    else if( msg.data == 2)
        zone = RETRY;
}






