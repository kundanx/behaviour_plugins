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
    action_client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(node_ptr_, "/navigate_through_poses");
    subscription_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToBallPose::odometry_callback,this,std::placeholders::_1)
    );
    done_flag = false;
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
    compute_goal();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToBallPose::nav_to_pose_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToBallPose::nav_to_pose_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // send pose
    done_flag = false;
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
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
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
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
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(node_ptr_->get_logger()," Navigating..");
}

void GoToBallPose::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    curr_pose = msg;
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
void GoToBallPose::compute_goal()
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



Quaternion ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


EulerAngles ToEulerAngles(double w, double x, double y, double z) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
