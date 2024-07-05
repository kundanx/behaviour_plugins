#include "conditions/isOnlyBall.hpp"   

isOnlyBall::isOnlyBall(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_only_ball", qos_profile, std::bind(&isOnlyBall::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"isOnlyBall::Ready..");
    onlyBall = false;
}
BT::PortsList isOnlyBall::providedPorts()
{
    return {BT::OutputPort<bool>("OP_IsOnlyBall")};
}

 BT::NodeStatus isOnlyBall::tick()
 {  
    if(node_called_once)
    {
        onlyBall = false;
        node_called_once = false;
    }
    if(onlyBall){
        RCLCPP_INFO(node_ptr_->get_logger(),"isOnlyBall::yes");
        setOutput<bool>("OP_IsOnlyBall", onlyBall);

        onlyBall = false;
        node_called_once = true;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"isOnlyBall::Multiple balls inside");
    return BT::NodeStatus::FAILURE;
 }

void isOnlyBall::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1)
        onlyBall = true;    
    else
        onlyBall = false;
}
