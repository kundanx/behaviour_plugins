#include "ResetOdom.hpp"   

ResetOdom::ResetOdom(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    RCLCPP_INFO(node_ptr_->get_logger(),"ResetOdom node Ready..");
}

BT::PortsList ResetOdom::providedPorts()
{
    return {BT::InputPort<int>("Ip_SiloNumber")};
}

 BT::NodeStatus ResetOdom::tick()
 {  
    auto silo_number_ = getInput<int>("Ip_SiloNumber");
    if( !silo_number_ )
    {
        throw BT::RuntimeError("error reading port [Ip_SiloNumber]:", silo_number_.error());
    }
    auto silo_number = silo_number_.value();


    std_msgs::msg::UInt8 msg;
    msg.data = silo_number;
    publisher_->publish(msg);
    return BT::NodeStatus::SUCCESS;
 }


