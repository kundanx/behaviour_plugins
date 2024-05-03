#include "GetBallPose.hpp"   

GetBallPose::GetBallPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>( 
        "/ball_pose_topic",
        10,
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
