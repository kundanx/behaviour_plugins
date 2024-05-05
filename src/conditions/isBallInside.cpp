#include "conditions/isBallInside.hpp"   

isBallInside::isBallInside(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "Ball_status", 10, std::bind(&isBallInside::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside node Ready..");
    inside = false;
}
BT::PortsList isBallInside::providedPorts()
{
    return {BT::OutputPort<bool>("OP_IsBallInside")};
}

 BT::NodeStatus isBallInside::tick()
 {  
    if(inside){
        RCLCPP_INFO(node_ptr_->get_logger(),"Ball Inside.");
        setOutput<bool>("OP_IsBallInside", inside);

        inside = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Ball not Inside.");
    return BT::NodeStatus::FAILURE;
 }

void isBallInside::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 1)
        inside = true;    
    else
        inside = false;
}
