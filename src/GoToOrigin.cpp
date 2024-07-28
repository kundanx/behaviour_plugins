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

    color_feedback_publisher = node_ptr_->create_publisher<std_msgs::msg::Int8>("color_feedback/GoToOrigin", qos_profile);

    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToOrigin::odometry_callback, this, std::placeholders::_1));
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
    color_feedback_publisher->publish(msg);
    
}

void GoToOrigin::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
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
    start_time = get_tick_ms();
    prev_x = odom_msg.pose.pose.position.x;
    prev_y = odom_msg.pose.pose.position.y;

    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToOrigin::sent goal");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToOrigin::onRunning()
{   
  
    if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.02 && (prev_y - odom_msg.pose.pose.position.y) < 0.02)
    {
        uint32_t now = get_tick_ms();
        if( now - start_time >= 1000)
        {
            cancel_goal();
            this->done_flag = true;
            RCLCPP_INFO(node_ptr_->get_logger(), " GoToSiloPose::Inside cancel ");

        }   
    }
    else
    {
        prev_x = odom_msg.pose.pose.position.x;
        prev_y = odom_msg.pose.pose.position.y;
        start_time = get_tick_ms();
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

