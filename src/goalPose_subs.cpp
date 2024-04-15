#include "goalPose_subs.hpp"   

GoalPoseSubs::GoalPoseSubs(const std::string &name,
                    rclcpp::Node::SharedPtr node_ptr) : BT::SyncActionNode(name,{}), node_ptr_(node_ptr)
{
    // publisher_ = node_ptr_->create_publisher<std_msgs::msg::UInt8>("/control_topic", 10);
    subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>( "/ball_pose_topic", 10, std::bind(&GoToPose::ball_pose_callback,this,std::placeholders::_1));
    // action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    
    done_flag = false;
}

BT::PortsList GoalPoseSubs::providedPorts()
{
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")};
}

 BT::NodeStatus GoalPoseSubs::tick()
 {  
    setOutput<PosMsg>("pose", PoseStamped);
    return BT::NodeStatus::SUCCESS;
 }


void GoalPoseSubs::ball_pose_callback(geometry_msgs::msg::PoseStamped msg)
{   
    poseStamped = msg;
    // xyz[0] = msg.pose.position.x;
    // xyz[1] = msg.pose.position.y;
    // xyz[2] = 0.0;

    // q[0]= msg.pose.orientation.x;
    // q[1]= msg.pose.orientation.y;
    // q[2]= msg.pose.orientation.z;
    // q[3]= msg.pose.orientation.w;
    
}
