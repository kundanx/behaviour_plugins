#include "GoToBallPose.h"   

/*****************************************************************************************************************
 * @brief Navigate to ball position after ball position is recieved 
******************************************************************************************************************/
GoToBallPose::GoToBallPose(
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
        std::bind(&GoToBallPose::odometry_callback,this,std::placeholders::_1)
    );
    subscription_team_color = node_ptr_->create_subscription<std_msgs::msg::Int8>(
        "team_color",
        qos_profile,
        std::bind(&GoToBallPose::team_color_callback, this, std::placeholders::_1)
    );

    done_flag = false;
    goal_sent_once = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToBallPose node Ready..");
}

BT::PortsList GoToBallPose::providedPorts()
{
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("in_pose"),
            BT::InputPort<int>("In_start_wait")};
}

BT::NodeStatus GoToBallPose::onStart()
{   
    nav_to_pose_compute_goal();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GoToBallPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToBallPose::goal_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToBallPose::goal_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToBallPose::sent goal");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToBallPose::onRunning()
{     
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToBallPose::onHalted() 
{
    cancel_goal();
}

void GoToBallPose::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoToBallPose::Navigate to ball pose rejected");
    }
    else
    {
        goal_sent_once = true;
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToBallPose::Navigating to ball pose ");
        this->goal_handle = goal_handle_;
    }

}


void GoToBallPose::goal_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
void GoToBallPose::goal_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}

void GoToBallPose::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    curr_pose = msg;
}

void GoToBallPose::team_color_callback(const std_msgs::msg::Int8 &msg)
{
    if( msg.data == -1)
        team_color = RED;
    else
        team_color = BLUE;
    
    auto start_wait_ = getInput<int>("In_start_wait");
    if(start_wait_ )
    {
        if(start_wait_.value() == -1)
        {
            if ( goal_handle)
                cancel_goal();
        }
    }
}

void GoToBallPose::cancel_goal()
{
    if (goal_handle )
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "GoToBallPose::No active goal to cancel");
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToBallPose::Goal canceled");
        try
        {
            auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        }
        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
        {
            RCLCPP_WARN(node_ptr_->get_logger(),"GoToBallPose::cancel_goal::rclcpp_action::exceptions::UnknownGoalHandleError");
        }
    }

}
void GoToBallPose::get_curve_points(double input_points[2][3], double output_points[5][3])
{
    float points_to_evaluate[5] = {0.2f, 0.4f, 0.5f, .6f, .8f};
    bezier::Bezier<3> curve{
        {{input_points[0][0], input_points[0][1]},
        {input_points[1][0], input_points[0][1]},
        {input_points[1][0], input_points[0][1]},
        {input_points[1][0], input_points[1][1]}}};
    bezier::Point point;
    for (int i = 0; i < 5; i++)
    {
        point = curve.valueAt(points_to_evaluate[i]);
        output_points[i][0] = point[0];
        output_points[i][1] = point[1];
        output_points[i][2] = input_points[0][2] + (0.2f * (i + 1)) * (input_points[1][2] - input_points[0][2]);
  }
}


void GoToBallPose::nav_to_pose_compute_goal()
{
    auto goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
    if( !goal_pose_ )
    {
        throw BT::RuntimeError("error reading port [pose]:", goal_pose_.error());
    }
    auto goal_pose = goal_pose_.value();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_pose.pose.position.x;
    goal_msg.pose.pose.position.y = goal_pose.pose.position.y;
    goal_msg.pose.pose.position.z = goal_pose.pose.position.z;

    goal_msg.pose.pose.orientation.x = goal_pose
    .pose.orientation.x;
    goal_msg.pose.pose.orientation.y = goal_pose
    .pose.orientation.y;
    goal_msg.pose.pose.orientation.z = goal_pose
    .pose.orientation.z;
    goal_msg.pose.pose.orientation.w = goal_pose
    .pose.orientation.w;

}

