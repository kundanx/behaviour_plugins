#include "decorator/returnFailureNode.hpp"

returnFailure::returnFailure(
    const std::string &name,
    const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name,config)
{
    RCLCPP_INFO(rclcpp::get_logger("returnFailure"),"returnFailure action Ready..");
}

BT::NodeStatus returnFailure::tick() 
{
    RCLCPP_INFO(rclcpp::get_logger("returnFailure"),"......");
    return BT::NodeStatus::FAILURE;
}