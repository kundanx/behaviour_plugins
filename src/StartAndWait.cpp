#include "StartAndWait.hpp"   

StartAndWait::StartAndWait(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "wait_and_go", qos_profile, std::bind(&StartAndWait::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::Ready..");
    wait = true;

}

 BT::NodeStatus StartAndWait::tick()
 {  
    if(!wait && start)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::Start");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::");
    return BT::NodeStatus::SUCCESS;
 }

void StartAndWait::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 0x0f)
    {
        start = true;    
        wait = false;
    }
    else if ( msg.data == 0xf0)
    {
        start = false;    
        wait = true;
    }
}
