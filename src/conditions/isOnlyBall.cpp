#include "conditions/isOnlyBall.hpp"   

isOnlyBall::isOnlyBall(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "/ColorSensor_status", 10, std::bind(&isOnlyBall::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"isOnlyBall node Ready..");
    onlyBall = false;
}
BT::PortsList isOnlyBall::providedPorts()
{
    return {BT::OutputPort<bool>("OP_IsOnlyBall")};
}

 BT::NodeStatus isOnlyBall::tick()
 {  
    if(onlyBall){
        RCLCPP_INFO(node_ptr_->get_logger(),"Only ball inside");
        setOutput<bool>("OP_IsOnlyBall", onlyBall);

        onlyBall = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Multiple balls inside");
    return BT::NodeStatus::FAILURE;
 }

void isOnlyBall::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 0xA2)
        onlyBall = true;    
    else
        onlyBall = false;
}
