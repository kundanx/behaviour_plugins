#include "decorator/returnSuccessNode.hpp"

returnSuccess::returnSuccess(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("returnSuccess"),"returnSuccess action Ready..");
}

BT::NodeStatus returnSuccess::tick() 
{
    RCLCPP_INFO(rclcpp::get_logger("returnSuccess"),"......");
    return BT::NodeStatus::SUCCESS;
}