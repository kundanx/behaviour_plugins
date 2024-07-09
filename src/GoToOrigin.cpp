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
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
     subscription_team_color = node_ptr_->create_subscription<std_msgs::msg::Int8>(
        "team_color",
        qos_profile,
        std::bind(&GoToOrigin::team_color_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin::Ready");
    done_flag = false;
}

void GoToOrigin::team_color_callback(const std_msgs::msg::Int8 &msg)
{
    if( msg.data == -1)
        team_color = RED;
    else
        team_color = BLUE;
}
 BT::PortsList GoToOrigin::providedPorts()
 {
    return{BT::InputPort<int>("In_start_wait")};
 }

BT::NodeStatus GoToOrigin::onStart()
{
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&GoToOrigin::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToOrigin::goal_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToOrigin::goal_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    auto goal_msg = NavigateToPose::Goal();

    Quaternion q;
    q = ToQuaternion(0.0, 0.0,  (1.57 * team_color));

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = q.x;
    goal_msg.pose.pose.orientation.y = q.y;
    goal_msg.pose.pose.orientation.z = q.z;
    goal_msg.pose.pose.orientation.w = q.w;

    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin::sent goal");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToOrigin::onRunning()
{   
    auto start_wait_ = getInput<int>("In_start_wait");
    if(start_wait_ )
    {
        if(start_wait_.value() == -1)
        {
            cancel_goal();
            return BT::NodeStatus::FAILURE;
        }
    }

    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToOrigin::onHalted() 
{
    cancel_goal();
    RCLCPP_WARN(node_ptr_->get_logger(),"GoToOrigin::Navigation aborted");
}

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

void GoToOrigin::goal_result_callback(const GoalHandleNav::WrappedResult &wrappedresult)
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
void GoToOrigin::goal_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
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

