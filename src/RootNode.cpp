#include "RootNode.hpp"   

/*****************************************************************************************************************
 * @brief BT Node which return true if ball is detected by the camera
 * @brief Subscribed topic : 'is_ball_tracked'
******************************************************************************************************************/

RootNode::RootNode(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = node_ptr_->create_subscription<action_pkg::msg::RobotConfig>( "/robot_configurations", qos_profile, std::bind(&RootNode::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"RootNode::Ready");
}

BT::PortsList RootNode::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("op_pose")};
}

 BT::NodeStatus RootNode::tick()
 {  
    if(isDetected.data){
        RCLCPP_INFO(node_ptr_->get_logger(),"RootNode::Detected.");
        isDetected.data = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"RootNode::not Detected.");
    return BT::NodeStatus::FAILURE;
 }

void RootNode::subscriber_callback(action_pkg::msg::RobotConfig ball)
{   
    if(ball.is_tracked.data)
    {
        isDetected.data = true;    
        setOutput<geometry_msgs::msg::PoseStamped>("op_pose", ball.goalpose);
    }
    else
        isDetected.data = false;
}
