#include "ResetOdom.hpp"   

/*****************************************************************************************************************
 * @brief BT node to reset odometery after a ball is succesfully stored
 * @brief Published topic : 'alignedSilo_number' [silo number aligned to, subscribed by landmark_pose_update node]
******************************************************************************************************************/

ResetOdom::ResetOdom(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    publisher_ = node_ptr->create_publisher<std_msgs::msg::UInt8>("alignedSilo_number", qos_profile);
    RCLCPP_INFO(node_ptr_->get_logger(),"ResetOdom node Ready..");
}

BT::PortsList ResetOdom::providedPorts()
{
    return {BT::InputPort<uint8_t>("Ip_SiloNumber")};
}

 BT::NodeStatus ResetOdom::tick()
 {  
    // RCLCPP_INFO(node_ptr_->get_logger(),"RESETODOM::Tick");
    auto silo_number_ = getInput<uint8_t>("Ip_SiloNumber");
    // RCLCPP_INFO(node_ptr_->get_logger(),"RESETODOM::here");
    if( !silo_number_ )
    {
        throw BT::RuntimeError("RESETODOM::error reading port [Ip_SiloNumber]:", silo_number_.error());
    }
    auto silo_number = silo_number_.value();

    // RCLCPP_INFO(node_ptr_->get_logger(),"RESETODOM::here2");
    std_msgs::msg::UInt8 msg;
    msg.data = silo_number;
    publisher_->publish(msg);
    RCLCPP_INFO(node_ptr_->get_logger(),"RESETODOM::odometry reset..");
    return BT::NodeStatus::SUCCESS;
 }


