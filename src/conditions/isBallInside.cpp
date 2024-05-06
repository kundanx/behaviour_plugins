#include "conditions/isBallInside.hpp"   

isBallInside::isBallInside(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("isBallInside"),"isBallInside node Ready..");
    inside = false;
}
BT::PortsList isBallInside::providedPorts()
{
    return {BT::InputPort<bool>("Ip_IsBallInside")};
}

 BT::NodeStatus isBallInside::tick()
 {  
    auto inside_ = getInput<bool>("Ip_IsBallInside");
    if(!inside_)
    {
        throw BT::RuntimeError("[isBallInside] error reading port");
    }
    inside = inside_.value();
    if(inside){
        RCLCPP_INFO(rclcpp::get_logger("isBallInside"),"Ball Inside.");
        inside = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(rclcpp::get_logger("isBallInside"),"Ball not Inside.");
    return BT::NodeStatus::FAILURE;
 }
