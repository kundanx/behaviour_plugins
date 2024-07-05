#include "GoToMiddle.hpp"   

/*****************************************************************************************************************
 * @brief Tree Node to bring robot back to origin to intake new ball after ball is stored in silo
******************************************************************************************************************/

GoToMiddle::GoToMiddle(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");

    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToMiddle::odometry_callback, this, std::placeholders::_1));
    // updated_goal_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>( "/goal_update", 10);

    RCLCPP_INFO(node_ptr_->get_logger(),"GoToMiddle::Ready");
    done_flag = false;
}
BT::PortsList GoToMiddle::providedPorts()
{
    return {BT::InputPort<int>("Ip_middle_type")};
}
BT::NodeStatus GoToMiddle::onStart()
{
    auto middle_type_ = getInput<int>("Ip_middle_type");
    if(!middle_type_)
    {
        throw BT::RuntimeError("error reading port [Ip_middle_type]:", middle_type_.error());

    }
    int middle_type = middle_type_.value();
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&GoToMiddle::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToMiddle::nav_to_pose_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToMiddle::nav_to_pose_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make pose
    auto goal_msg = NavigateToPose::Goal();

    switch (middle_type)
    {
        case MIDDLE_OF_SLOPE:
            if ( fabs(odom_msg.pose.pose.position.x) > 1.3 )
                goal_msg.pose.pose.position.x = 1.3;
            else    
                goal_msg.pose.pose.position.x = odom_msg.pose.pose.position.x;
            goal_msg.pose.pose.position.y = -1.0;
            break;

        case ABSOLUTE_MIDDLE:
            goal_msg.pose.pose.position.x = 0.0;
            goal_msg.pose.pose.position.y = -2.0;
            break;
    }
        
    goal_msg.pose.header.frame_id = "map";
    
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.7071068;
    goal_msg.pose.pose.orientation.w = 0.7071068;

    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();

    // send pose
    done_flag = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToMiddle::sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToMiddle::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToMiddle::onHalted() 
{
    cancel_goal();
    RCLCPP_WARN(node_ptr_->get_logger(),"GoToMiddle::Navigation aborted");
}



void GoToMiddle::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
}
/*****************************************************************************************************************
 * @brief Cancle navigation action
 ******************************************************************************************************************/

void GoToMiddle::cancel_goal()
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

void GoToMiddle::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &wrappedresult)
{
    if(wrappedresult.result)
    {
        done_flag=true;
        RCLCPP_INFO(node_ptr_->get_logger()," GoToMiddle:: Result sucessfull");

    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger()," GoToMiddle:: Result ERROR..");

    }
}
void GoToMiddle::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
    // RCLCPP_INFO(node_ptr_->get_logger()," GoToMiddle::Navigating to origin..");
}

void GoToMiddle::goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_)
{
    if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoToMiddle::Nav to origin goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToMiddle::Nav to origin Goal accepted ");
        this->goal_handle = goal_handle_;
    }
}
