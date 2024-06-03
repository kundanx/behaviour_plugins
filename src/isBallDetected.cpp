#include "isBallDetected.hpp"   


/*****************************************************************************************************************
 * @brief BT Node which return true if ball is detected by the camera
 * @brief Subscribed topic : 'is_ball_tracked'
******************************************************************************************************************/

isBallDetected::isBallDetected(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::Bool>( "/is_ball_tracked", qos_profile, std::bind(&isBallDetected::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"IsBallDetected node Ready..");
    isDetected.data = false;
}

BT::PortsList isBallDetected::providedPorts()
{
    return {BT::OutputPort<std_msgs::msg::Bool>("op_ballDetectionFlag")};
}

 BT::NodeStatus isBallDetected::tick()
 {  
    if(isDetected.data){
        RCLCPP_INFO(node_ptr_->get_logger(),"Ball Detected.");
        setOutput<std_msgs::msg::Bool>("op_ballDetectionFlag", isDetected);
        isDetected.data = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Ball not Detected.");
    return BT::NodeStatus::FAILURE;
 }

void isBallDetected::subscriber_callback(std_msgs::msg::Bool msg)
{   
    if(msg.data)
        isDetected.data = true;    
    else
        isDetected.data = false;
}
