#include "goalPose_subscriber.hpp"

RecieveGoalPose::RecieveGoalPose(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
:RosTopicSubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{}

BT::PortsList RecieveGoalPose::providedPorts()
{
    return providedBasicPorts({OutputPort<std::shared_ptr<PosMsg>>("pose")});
}

BT::NodeStatus RecieveGoalPose::onTick(const std::shared_ptr<PosMsg>& new_goal) 
{    
    RCLCPP_INFO(logger(),"Inside onTick");
    if(new_goal)
    {
        RCLCPP_INFO(logger(), "[%s] new message: Goal recieved ",name().c_str());
        // pose_.header.frame_id = "map";

        // pose_.pose.position.x = new_goal->pose.position.x;
        // pose_.pose.position.y = new_goal->pose.position.y;
        // pose_.pose.position.z = new_goal->pose.position.z;

        // pose_.pose.orientation.x = new_goal->pose.orientation.x;
        // pose_.pose.orientation.y = new_goal->pose.orientation.y;
        // pose_.pose.orientation.z = new_goal->pose.orientation.z;
        // pose_.pose.orientation.w = new_goal->pose.orientation.w;
        RCLCPP_INFO(logger(),"Inside on newgoal");
        setOutput("pose", new_goal );
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}
