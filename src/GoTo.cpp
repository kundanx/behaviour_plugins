#include "GoTo.hpp"   

/*****************************************************************************************************************
 * @brief Navigate to ball position after ball position is recieved 
******************************************************************************************************************/
GoTo::GoTo(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");

    color_feedback_publisher = node_ptr_->create_publisher<std_msgs::msg::Int8>("color_feedback/GoTo", qos_profile);

    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoTo::odometry_callback,this,std::placeholders::_1)
    );
 
    subscription_team_color = node_ptr_->create_subscription<std_msgs::msg::Int8>(
        "team_color",
        qos_profile,
        std::bind(&GoTo::team_color_callback, this, std::placeholders::_1)
    );

    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoTo node Ready..");
}

BT::PortsList GoTo::providedPorts()
{
    return {BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("theta")};
}

BT::NodeStatus GoTo::onStart()
{  

    nav_to_pose_compute_goal();
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GoTo::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoTo::goal_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoTo::goal_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    prev_x = odom_msg.pose.pose.position.x;
    prev_y = odom_msg.pose.pose.position.y;
    start_time = get_tick_ms();

    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoTo::sent goal");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoTo::onRunning()
{     

    uint32_t now = get_tick_ms();
    if ( now - start_time >= 3000)
    {
        if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.01 && fabs((prev_y - odom_msg.pose.pose.position.y)) < 0.01)
        {
            cancel_goal();
            this->done_flag = true;
            RCLCPP_INFO(node_ptr_->get_logger(), " RecoveryNode::Robot static.. cancel goal ");
            return BT::NodeStatus::SUCCESS; 

        }
        else
        {
            start_time = get_tick_ms();
        }
        prev_x = odom_msg.pose.pose.position.x;
        prev_y = odom_msg.pose.pose.position.y;
    }

    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoTo::onHalted() 
{
    cancel_goal();
}

void GoTo::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoTo::Navigate to ball pose rejected");
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoTo::Navigating to given pose ");
        this->goal_handle = goal_handle_;
    }
}

void GoTo::goal_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
void GoTo::goal_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}


void GoTo::team_color_callback(const std_msgs::msg::Int8 &msg)
{
    if( msg.data == -1)
        team_color = RED;
    else
        team_color = BLUE;
    
    color_feedback_publisher->publish(msg);
}

void GoTo::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    odom_msg = msg;
}

void GoTo::cancel_goal()
{
    if (goal_handle )
    {
        try
        {
            auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        }
        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
        {
            RCLCPP_WARN(node_ptr_->get_logger(),"GoTo::cancel_goal::rclcpp_action::exceptions::UnknownGoalHandleError");
        }
        RCLCPP_INFO(node_ptr_->get_logger(), "GoTo::Goal canceled");

    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "GoTo::No active goal to cancel");
       
    }
}

void GoTo::nav_to_pose_compute_goal()
{
    auto x_ = getInput<float>("x");
    auto y_ = getInput<float>("y");
    auto theta_ = getInput<float>("theta");

    if( !x_ )
    {
        throw BT::RuntimeError("error reading port [x]:", x_.error());
    }

    if( !y_ )
    {
        throw BT::RuntimeError("error reading port [y]:", y_.error());
    }

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x_.value();
    goal_msg.pose.pose.position.y = y_.value();
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
}

