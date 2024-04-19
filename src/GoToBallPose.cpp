#include "GoToBallPose.h"   

GoToBallPose::GoToBallPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToBallPose node created..");
    done_flag = false;
}

BT::PortsList GoToBallPose::providedPorts()
{
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("in_pose")};
}

BT::NodeStatus GoToBallPose::onStart()
{
    // BT::Expected<std::string> loc = getInput<std::string>("loc");
    // const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    // YAML::Node locations = YAML::LoadFile(location_file);
    // std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();


    auto goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
    if( !goal_pose_ )
    {
        throw BT::RuntimeError("error reading port [pose]:", goal_pose_.error());
    }
    auto goal_pose = goal_pose_.value();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToBallPose::nav_to_pose_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToBallPose::nav_to_pose_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make pose
    auto goal_msg = NavigateToPose::Goal();
    
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_pose.pose.position.x;
    goal_msg.pose.pose.position.y = goal_pose.pose.position.y;
    goal_msg.pose.pose.position.z = goal_pose.pose.position.z;

    goal_msg.pose.pose.orientation.x = goal_pose.pose.orientation.x;
    goal_msg.pose.pose.orientation.y = goal_pose.pose.orientation.y;
    goal_msg.pose.pose.orientation.z = goal_pose.pose.orientation.z;
    goal_msg.pose.pose.orientation.w = goal_pose.pose.orientation.w;

    // send pose
    done_flag = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToBallPose::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToBallPose::onHalted() 
{
     RCLCPP_WARN(node_ptr_->get_logger(),"Navigation aborted");
}

void GoToBallPose::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
void GoToBallPose::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(node_ptr_->get_logger()," Navigating..");
}
