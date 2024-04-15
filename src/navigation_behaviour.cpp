#include "navigation_behaviour.h"   

GoToPose::GoToPose(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    // publisher_ = node_ptr_->create_publisher<std_msgs::msg::UInt8>("/control_topic", 10);
    // subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::Pose>( "/ball_pose_topic", 10, std::bind(&GoToPose::ball_pose_callback,this,std::placeholders::_1));
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    
    done_flag = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::shared_ptr<PosMsg>>("pose")};
}

BT::NodeStatus GoToPose::onStart()
{
    // BT::Expected<std::string> loc = getInput<std::string>("loc");
    // const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    // YAML::Node locations = YAML::LoadFile(location_file);
    // std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();


    auto goal_pose_ = getInput<std::shared_ptr<PosMsg>>("pose");
    if( !goal_pose_ )
    {
        throw BT::RuntimeError("error reading port [target]:", goal_pose_.error());
    }
    auto goal_pose = goal_pose_.value();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_result_callback, this, std::placeholders::_1);
    // send_goal_options.feedback_callback = std::bind(&GoToPose::nav_to_pose_feedback_callback, this, std::placeholders::_1);

    // make pose
    auto goal_msg = NavigateToPose::Goal();
    
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_pose->pose.position.x;
    goal_msg.pose.pose.position.y = goal_pose->pose.position.y;
    goal_msg.pose.pose.position.z = goal_pose->pose.position.z;

    // tf2::Quaternion q;
    // q.setRPY(0,0,xy_theta[2]);
    // q.normalize();

    goal_msg.pose.pose.orientation.x = goal_pose->pose.orientation.x;
    goal_msg.pose.pose.orientation.y = goal_pose->pose.orientation.y;
    goal_msg.pose.pose.orientation.z = goal_pose->pose.orientation.z;
    goal_msg.pose.pose.orientation.w = goal_pose->pose.orientation.w;

    // send pose
    done_flag = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToPose::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
void GoToPose::onHalted() 
{
    printf("Navigation Aborted. \n");
}
void GoToPose::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
// void GoToPose::nav_to_pose_feedback_callback(const GoalHandleNav::Feedback feedback)
// {
//     // std::stringstream ss;
//     // ss<<feedback.current_pose;
//     RCLCPP_INFO(node_ptr_->get_logger()," Navigating..");

// }
// void GoToPose::ball_pose_callback(const geometry_msgs::msg::Pose & msg)
// {
//     xyz[0] = msg.position.x;
//     xyz[1] = msg.position.y;
//     xyz[2] = 0.0;

//     q[0]= msg.orientation.x;
//     q[1]= msg.orientation.y;
//     q[2]= msg.orientation.z;
//     q[3]= msg.orientation.w;
// }
