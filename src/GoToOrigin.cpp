#include "GoToOrigin.hpp"   

/*****************************************************************************************************************
 * @brief Tree Node to bring robot back to origin to intake new ball after ball is stored in silo
******************************************************************************************************************/

GoToOrigin::GoToOrigin(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin node Ready..");
    done_flag = false;
}

BT::NodeStatus GoToOrigin::onStart()
{
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToOrigin::nav_to_pose_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToOrigin::nav_to_pose_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make pose
    auto goal_msg = NavigateToPose::Goal();
        
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.7071068;
    goal_msg.pose.pose.orientation.w = 0.7071068;
    // send pose
    done_flag = false;
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToOrigin::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToOrigin::onHalted() 
{
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    RCLCPP_WARN(node_ptr_->get_logger(),"Navigation aborted");
}

void GoToOrigin::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
void GoToOrigin::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(node_ptr_->get_logger()," Navigating to origin..");
}
