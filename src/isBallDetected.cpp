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
    subscription_ = node_ptr_->create_subscription<action_pkg::msg::BallPose>( "/ball_tracker", qos_profile, std::bind(&isBallDetected::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"IsBallDetected node Ready..");
    isDetected.data = false;
}

BT::PortsList isBallDetected::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("op_pose")};
}

 BT::NodeStatus isBallDetected::tick()
 {  
    if(isDetected.data){
        RCLCPP_INFO(node_ptr_->get_logger(),"Ball Detected.");
        isDetected.data = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Ball not Detected.");
    return BT::NodeStatus::FAILURE;
 }

void isBallDetected::subscriber_callback(action_pkg::msg::BallPose ball)
{   
    if(ball.is_tracked.data)
    {
        isDetected.data = true;    
        setOutput<geometry_msgs::msg::PoseStamped>("op_pose", ball.goalpose);
    }
    else
        isDetected.data = false;
}
