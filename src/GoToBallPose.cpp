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
    // action_client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(node_ptr_, "/navigate_through_poses");
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");

    subscription_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToBallPose::odometry_callback,this,std::placeholders::_1)
    );
    // ball_track_subscription_ = node_ptr_->create_subscription<oakd_msgs::msg::StatePose>(
    //     "/ball_tracker",
    //     qos_profile, 
    //     std::bind(&GoToBallPose::ball_tracker_callback,this,std::placeholders::_1)
    // );

    updated_goal_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>( "/goal_update", 10);


    done_flag = false;
    goal_sent_once = false;
    goal_msg.poses.resize(5);
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToBallPose node Ready..");
}

BT::PortsList GoToBallPose::providedPorts()
{
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("in_pose")};
}

BT::NodeStatus GoToBallPose::onStart()
{   
    // make poses
    // nav_through_pose_compute_goal();
    nav_to_pose_compute_goal();
    // cancel_goal();

    // Setup action client goal
    // auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&GoToBallPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToBallPose::goal_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToBallPose::goal_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // send pose
    // action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    action_client_ptr_->async_send_goal(goal_msg_to, send_goal_options);

    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToBallPose::sent goal to nav2");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToBallPose::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    nav_to_pose_compute_goal();
    return BT::NodeStatus::RUNNING;
}

void GoToBallPose::onHalted() 
{
    cancel_goal();
    // auto cancel_future= action_client_ptr_->async_cancel_goal(goal_handle);
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
    // RCLCPP_INFO(node_ptr_->get_logger()," Navigating..");
}

void GoToBallPose::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    curr_pose = msg;
}

void GoToBallPose::ball_tracker_callback(oakd_msgs::msg::StatePose ball)
{
     
    if(!ball.is_tracked.data)
    {
        if(goal_handle)
        {   
            RCLCPP_INFO(node_ptr_->get_logger()," here..");
            // cancel_goal();
            // auto cancel_future= action_client_ptr_->async_cancel_goal(goal_handle);
        }
    }
  
}
void GoToBallPose::cancel_goal()
{
    if (goal_handle == nullptr)
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
void GoToBallPose::nav_through_pose_compute_goal()
{
    auto goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
    if( !goal_pose_ )
    {
        throw BT::RuntimeError("error reading port [pose]:", goal_pose_.error());
    }
    auto goal_pose = goal_pose_.value();

    EulerAngles e;
    e = ToEulerAngles(curr_pose.pose.pose.orientation.w,
                      curr_pose.pose.pose.orientation.x,
                      curr_pose.pose.pose.orientation.y,
                      curr_pose.pose.pose.orientation.z);
    double curr_yaw = e.yaw;

    e = ToEulerAngles(goal_pose.pose.orientation.w,
                      goal_pose.pose.orientation.x,
                      goal_pose.pose.orientation.y,
                      goal_pose.pose.orientation.z);
    double goal_yaw = e.yaw;

    double input_points[2][3] = {{curr_pose.pose.pose.position.x, curr_pose.pose.pose.position.y, curr_yaw}, 
                                {goal_pose.pose.position.x, goal_pose.pose.position.y, goal_yaw}};
    double output_points[5][3];

    get_curve_points(input_points, output_points);

    for( int i = 0; i< 4; i++)
    {

        goal_msg.poses[i].header.frame_id = "map";
        goal_msg.poses[i].pose.position.x =  output_points[i][0];
        goal_msg.poses[i].pose.position.y =  output_points[i][1];
        goal_msg.poses[i].pose.position.z = 0.0;

        Quaternion q;
        q = ToQuaternion(0.0, 0.0, output_points[i][2]);
        goal_msg.poses[i].pose.orientation.x = q.x;
        goal_msg.poses[i].pose.orientation.y = q.y;
        goal_msg.poses[i].pose.orientation.z = q.z;
        goal_msg.poses[i].pose.orientation.w = q.w;
    }

    goal_msg.poses[4].header.frame_id = "map";
    goal_msg.poses[4].pose.position.x =  goal_pose.pose.position.x;
    goal_msg.poses[4].pose.position.y =  goal_pose.pose.position.y;
    goal_msg.poses[4].pose.position.z = 0.0;

    goal_msg.poses[4].pose.orientation.x = goal_pose.pose.orientation.x;
    goal_msg.poses[4].pose.orientation.y = goal_pose.pose.orientation.y;
    goal_msg.poses[4].pose.orientation.z = goal_pose.pose.orientation.z;
    goal_msg.poses[4].pose.orientation.w = goal_pose.pose.orientation.w; 
}

void GoToBallPose::nav_to_pose_compute_goal()
{
    auto goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
    if( !goal_pose_ )
    {
        throw BT::RuntimeError("error reading port [pose]:", goal_pose_.error());
    }
    auto goal_pose = goal_pose_.value();


    double del_y =  goal_pose.pose.position.y - curr_pose.pose.pose.position.y;
    double del_x = fabs(goal_pose.pose.position.x - curr_pose.pose.pose.position.x);

    if ( del_x > 0.5)
        del_x = 0.5; 

    del_x = del_x * 2;

    double updated_y = goal_pose.pose.position.y - del_y * del_x;

    double input_points[2][3] = {{curr_pose.pose.pose.position.x, curr_pose.pose.pose.position.y, 0.0}, 
                                {goal_pose.pose.position.x, goal_pose.pose.position.y, 0.0}};
    double output_points[5][3];

    get_curve_points(input_points, output_points);

    geometry_msgs::msg::PoseStamped updated_goal;
    updated_goal.header.frame_id = "map";
    // updated_goal.pose.position.x = output_points[0][0];
    updated_goal.pose.position.x = goal_pose.pose.position.x;
    // updated_goal.pose.position.y = output_points[0][1];
    // updated_goal.pose.position.y = updated_y;
    updated_goal.pose.position.y =goal_pose.pose.position.y;
    updated_goal.pose.position.z = goal_pose.pose.position.z;

    updated_goal.pose.orientation.x = goal_pose.pose.orientation.x;
    updated_goal.pose.orientation.y = goal_pose.pose.orientation.y;
    updated_goal.pose.orientation.z = goal_pose.pose.orientation.z;
    updated_goal.pose.orientation.w = goal_pose.pose.orientation.w;

    goal_msg_to.pose.header.frame_id = "map";
    goal_msg_to.pose.pose.position.x = updated_goal.pose.position.x;
    goal_msg_to.pose.pose.position.y = updated_goal.pose.position.y;
    goal_msg_to.pose.pose.position.z = updated_goal.pose.position.z;

    goal_msg_to.pose.pose.orientation.x = updated_goal.pose.orientation.x;
    goal_msg_to.pose.pose.orientation.y = updated_goal.pose.orientation.y;
    goal_msg_to.pose.pose.orientation.z = updated_goal.pose.orientation.z;
    goal_msg_to.pose.pose.orientation.w = updated_goal.pose.orientation.w;



    updated_goal_publisher_->publish(updated_goal);

}

