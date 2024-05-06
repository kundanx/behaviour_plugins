#include "conditions/isOnlyBall.hpp"   

isOnlyBall::isOnlyBall(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("isOnlyBall"),"isOnlyBall node Ready..");
    onlyBall = false;
}
BT::PortsList isOnlyBall::providedPorts()
{
    return {BT::InputPort<bool>("Ip_IsOnlyBall")};
}

 BT::NodeStatus isOnlyBall::tick()
 {  
    auto onlyBall_ = getInput<bool>("Ip_IsOnlyBall");
    if(!onlyBall_)
    {
        throw BT::RuntimeError("[isOnlyBall] error reading port");
    }
    onlyBall = onlyBall_.value();
    if(onlyBall){
        RCLCPP_INFO(rclcpp::get_logger("isOnlyBall"),"Only ball inside");
        onlyBall = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(rclcpp::get_logger("isOnlyBall"),"Multiple balls inside");
    return BT::NodeStatus::FAILURE;
 }

