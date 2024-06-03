/*******************************************************************************************************************
 * @brief BT node to verify if the team color ball is detected by camera
 * @brief Subscribed topic :- ball_pose_topic
 * @brief output port :- op_pose={pose}
******************************************************************************************************************/

#include "GetBallPose.hpp"   

GetBallPose::GetBallPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>( 
        "/ball_pose_topic",
        qos_profile,
        std::bind(&GetBallPose::ball_pose_callback,this,std::placeholders::_1)
    );
    RCLCPP_INFO(node_ptr_->get_logger(),"GetBallPose node Ready..");
    pose_recieved_flag = false;
}

BT::PortsList GetBallPose::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("op_pose")};
}

 BT::NodeStatus GetBallPose::tick()
 {  
    if(pose_recieved_flag){
        setOutput<geometry_msgs::msg::PoseStamped>("op_pose", ballPose);
        pose_recieved_flag = false;
        // RCLCPP_INFO(node_ptr_->get_logger(),"written in port [pose/pose]..");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_ptr_->get_logger(),"No pose recieved");
    return BT::NodeStatus::FAILURE;
 }

void GetBallPose::ball_pose_callback(geometry_msgs::msg::PoseStamped msg)
{   
    ballPose = msg;
    pose_recieved_flag = true;
}
