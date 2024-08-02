#include "actions/StartPhotoCapture.hpp"   

StartPhotoCapture::StartPhotoCapture(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::UInt8>( "Take_pictures", 10);
    RCLCPP_INFO(node_ptr_->get_logger(),"StartPhotoCapture node Ready..");
}

BT::PortsList StartPhotoCapture::providedPorts()
{
    return {
        BT::InputPort<int>("Ip_camera_type")  
    };
}

 BT::NodeStatus StartPhotoCapture::tick()
 {  

    auto camera_type_ = getInput<bool>("Ip_camera_type");
    std_msgs::msg::UInt8 msg;
    msg.data = 0;
    
    if(camera_type_)
    {
        msg.data = camera_type_.value();
    }
    
    publisher_->publish(msg);

    if ( msg.data == 1)
        RCLCPP_INFO(rclcpp::get_logger("StartPhotoCapture"),"Take pictures.. Oakd");
    else if ( msg.data == 2)
        RCLCPP_INFO(rclcpp::get_logger("StartPhotoCapture"),"Take pictures.. Silo");
    else
        RCLCPP_INFO(rclcpp::get_logger("StartPhotoCapture"),"Stop taking pictures");


    return BT::NodeStatus::SUCCESS;
 }

















