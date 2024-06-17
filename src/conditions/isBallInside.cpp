#include "conditions/isBallInside.hpp"   

isBallInside::isBallInside(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_ball_inside", qos_profile, std::bind(&isBallInside::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::Ready");
    inside = false;
}
BT::PortsList isBallInside::providedPorts()
{
    return {BT::OutputPort<bool>("OP_IsBallInside")};
}

 BT::NodeStatus isBallInside::tick()
 {  
    if(node_called_once)
    {
        inside = false;
        node_called_once = false;
    }
    if(inside){
        RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::yes");
        setOutput<bool>("OP_IsBallInside", inside);

        inside = false;
        node_called_once = true;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::Ball not Inside.");
    return BT::NodeStatus::FAILURE;
 }

void isBallInside::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1)
        inside = true;    
    else
        inside = false;
}
