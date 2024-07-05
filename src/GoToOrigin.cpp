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
    updated_goal_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>( "/goal_update", 10);

    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin::Ready");
    done_flag = false;
}

BT::NodeStatus GoToOrigin::onStart()
{
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&GoToOrigin::goal_response_callback, this, std::placeholders::_1);
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

    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();

    // send pose
    done_flag = false;
    compute_goal_NavTo();
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin::sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToOrigin::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    compute_goal_NavTo();
    return BT::NodeStatus::RUNNING;
}

void GoToOrigin::onHalted() 
{
    cancel_goal();
    RCLCPP_WARN(node_ptr_->get_logger(),"GoToOrigin::Navigation aborted");
}

/*****************************************************************************************************************
 * @brief Cancle navigation action
 ******************************************************************************************************************/

void GoToOrigin::cancel_goal()
{
    if (goal_handle)
    {
       try
        {
            auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        }
        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
        {
            RCLCPP_WARN(node_ptr_->get_logger(),"GoToSiloPose::cancel_goal::rclcpp_action::exceptions::UnknownGoalHandleError");
        }
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Goal canceled");
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "GoToSiloPose::No active goal to cancel");
    }
}

void GoToOrigin::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &wrappedresult)
{
    if(wrappedresult.result)
    {
        done_flag=true;
        RCLCPP_INFO(node_ptr_->get_logger()," GoToOrigin:: Result sucessfull");

    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger()," GoToOrigin:: Result ERROR..");

    }
}
void GoToOrigin::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
    // RCLCPP_INFO(node_ptr_->get_logger()," GoToOrigin::Navigating to origin..");
}

void GoToOrigin::goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_)
{
    if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoToOrigin::Nav to origin goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToOrigin::Nav to origin Goal accepted ");
        this->goal_handle = goal_handle_;
    }
}

void GoToOrigin::compute_goal_NavTo()
{
    geometry_msgs::msg::PoseStamped updated_goal;

    updated_goal.header.frame_id = "map";
    updated_goal.pose.position.x = 0.0;
    updated_goal.pose.position.y = 0.0;
    updated_goal.pose.position.z = 0.0;

    updated_goal.pose.orientation.x = 0.0;
    updated_goal.pose.orientation.y = 0.0;
    updated_goal.pose.orientation.z = 0.7071068;
    updated_goal.pose.orientation.w = 0.7071068;

    updated_goal_publisher_->publish(updated_goal);

}
