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
    subscription_ = node_ptr_->create_subscription<oakd_msgs::msg::StatePose>( "/ball_tracker", qos_profile, std::bind(&isBallDetected::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"IsBallDetected::Ready");
    ball_not_detected_counter = MAX_NOT_DETECTED;
    isDetected.data = false;
}

BT::PortsList isBallDetected::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("op_pose")};
}

 BT::NodeStatus isBallDetected::tick()
 {  
    if(isDetected.data){
        RCLCPP_INFO(node_ptr_->get_logger(),"IsBallDetected::Detected.");
        isDetected.data = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"IsBallDetected::not Detected.");
    // rclcpp_action::Client::SharedPtr::async_cancel_all_goals();

    return BT::NodeStatus::FAILURE;
 }

void isBallDetected::subscriber_callback(oakd_msgs::msg::StatePose msg)
{   
    if(msg.is_tracked.data)
    {

        isDetected.data = true;    
        setOutput<geometry_msgs::msg::PoseStamped>("op_pose", msg.goalpose);
        // ball_not_detected_counter = 0;
        // ball = msg;
    }
    else
    {
        isDetected.data = false;
        // ball_not_detected_counter ++;
    }
    
    // if (ball_not_detected_counter < MAX_NOT_DETECTED)
    // {
    //     isDetected.data = true;    
    //     setOutput<geometry_msgs::msg::PoseStamped>("op_pose", ball.goalpose);
    // }
    // else 
    //     isDetected.data = false;

}
