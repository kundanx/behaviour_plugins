#include "GoToSiloPose.hpp"
#include <math.h>

using namespace std::chrono_literals;

/**********************************************************************************************************************
 * @brief ActionClient BT node to navigate to silo
 * @brief Subscribed topics : silo_number [silo to navigate to]
 *                          : junction_type [Abort navigation when landmark is reached and silo align action is called]
 ************************************************************************************************************************/

GoToSiloPose::GoToSiloPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    action_client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(node_ptr_, "/navigate_through_poses");
    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToSiloPose::odometry_callback, this, std::placeholders::_1));
    subscription_silonumber = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
        "/silo_number",
        qos_profile,
        std::bind(&GoToSiloPose::silo_subscriber_callback, this, std::placeholders::_1));
    subscription_junctiontype = node_ptr_->create_subscription<std_msgs::msg::UInt8>(
        "/junction_type",
        qos_profile,
        std::bind(&GoToSiloPose::junction_subscriber_callback, this, std::placeholders::_1));
    subscription_isOnLine = node_ptr_->create_subscription<std_msgs::msg::Bool>(
        "is_on_line",
        qos_profile,
        std::bind(&GoToSiloPose::line_subscriber_callback, this, std::placeholders::_1));

    // goal_abort_timer = node_ptr_->create_wall_timer(
    //     1ms, std::bind(&GoToSiloPose::goal_abort_callback, this));

    done_flag = false;
    this->silo_number = 0;
    this->x_horiz_line_detected = false;
    goal_msg.poses.resize(2);

    RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Ready");
}

void GoToSiloPose::silo_subscriber_callback(const std_msgs::msg::UInt8 &msg)
{
    // RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose callback..");
    this->silo_number = msg.data;
}
void GoToSiloPose::junction_subscriber_callback(const std_msgs::msg::UInt8 &msg)
{
    if (msg.data == X_HORIZONTAL_LINE)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose:: x_horz_line_detected..");
        this->x_horiz_line_detected = true;
    }
}
void GoToSiloPose::line_subscriber_callback(const std_msgs::msg::Bool &msg)
{
    // if( msg.data)
    //     RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose:: is on line");
    this->is_on_line = msg.data;
}
void GoToSiloPose::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
}

BT::PortsList GoToSiloPose::providedPorts()
{
    return {BT::InputPort<int>("Ip_SiloNumber"),
            BT::OutputPort<uint8_t>("Op_SiloNumber")};
}

BT::NodeStatus GoToSiloPose::onStart()
{

    auto silo_number_ = getInput<int>("Ip_SiloNumber");
    if (!silo_number_)
    {
        throw BT::RuntimeError("GoToSiloPose:: error reading port [Ip_SiloNumber]:", silo_number_.error());
    }
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    this->testing = locations["testing"].as<bool>();

    if (testing)
    {
        this->silo_number = silo_number_.value();
    }

    // compute goal poses
    compute_goal_poses();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GoToSiloPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToSiloPose::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToSiloPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();

    done_flag = false;
    x_horiz_line_detected = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::sent goal to nav2\n");

    // update silo_number
    setOutput<uint8_t>("Op_SiloNumber", silo_number);

    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToSiloPose::onRunning()
{
    if (this->x_horiz_line_detected || odom_msg.pose.pose.position.y <= -2.20)
    {
        if (this->is_on_line)
        {
            cancel_goal();
            this->done_flag = true;
            this->is_on_line = false;
            this->x_horiz_line_detected = false;
            RCLCPP_INFO(node_ptr_->get_logger(), " GoToSiloPose::Inside cancel ");
        }
    }
    if (done_flag)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToSiloPose::onHalted()
{
    RCLCPP_WARN(node_ptr_->get_logger(), "GoToSiloPose::Navigation aborted");
}

void GoToSiloPose::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoToSiloPose::Navigate to silo %i goal rejected by server", this->silo_number);
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Navigating to silo %i ", this->silo_number);
        this->goal_handle = goal_handle_;
    }
}

void GoToSiloPose::result_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::[%s] Goal reached\n", this->name().c_str());
        done_flag = true;
    }
}

/*****************************************************************************************************************
 * @brief Received feedback from server
 * @brief Abort navigation when robot crosses X_HORZ line and y-pose threashold
 ******************************************************************************************************************/

void GoToSiloPose::feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    (void)feedback;

    // RCLCPP_INFO(node_ptr_->get_logger()," GoToSiloPose::Navigating to silo %i",this->silo_number);
}

/*****************************************************************************************************************
 * @brief Cancle navigation action
 ******************************************************************************************************************/

void GoToSiloPose::cancel_goal()
{
    if (goal_handle)
    {
        auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Goal canceled");
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "GoToSiloPose::No active goal to cancel");
    }
}

void GoToSiloPose::compute_goal_poses()
{
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    YAML::Node locations = YAML::LoadFile(location_file);

    ROBOT_POSE_WRT_SILO LRC = CENTRE;
    float x_waypoint_offset = 0.3;

    switch (this->silo_number)
    {
    case 1:
        this->pose = locations["silo_one"].as<std::vector<float>>();
        break;
    case 2:
        this->pose = locations["silo_two"].as<std::vector<float>>();
        break;
    case 3:
        this->pose = locations["silo_three"].as<std::vector<float>>();
        break;
    case 4:
        this->pose = locations["silo_four"].as<std::vector<float>>();
        break;
    case 5:
        this->pose = locations["silo_five"].as<std::vector<float>>();
        break;

    default:
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::silo number not recieved..[Default to zero]");
    }

    if (odom_msg.pose.pose.position.x < pose[0])
        LRC = LEFT;
    else if (odom_msg.pose.pose.position.x > pose[0])
        LRC = RIGHT;
    else
        LRC = CENTRE;

    float del_x = fabs(fabs(odom_msg.pose.pose.position.x) - fabs(pose[0]));
    float del_y = 1.036; // 2.2

    if (del_x > x_waypoint_offset)
    {
        if (LRC == LEFT)
        {
            this->pose_waypoint[0] = pose[0] - x_waypoint_offset;
            this->pose_waypoint[1] = pose[1] + del_y;
        }
        else if (LRC == RIGHT)
        {
            this->pose_waypoint[0] = pose[0] + x_waypoint_offset;
            this->pose_waypoint[1] = pose[1] + del_y;
        }
        else
        {
            this->pose_waypoint[0] = pose[0];
            this->pose_waypoint[1] = pose[1] + del_y;
        }
    }
    else
    {
        this->pose_waypoint[0] = odom_msg.pose.pose.position.x;
        this->pose_waypoint[1] = pose[1] + del_y;
    }

    goal_msg.poses[0].pose.position.x = pose_waypoint[0];
    goal_msg.poses[0].pose.position.y = pose_waypoint[1];

    goal_msg.poses[1].pose.position.x = pose[0];
    goal_msg.poses[1].pose.position.y = pose[1];

    for (int i = 0; i < 2; i++)
    {
        goal_msg.poses[i].header.frame_id = "map";
        goal_msg.poses[i].pose.position.z = 0.0;

        goal_msg.poses[i].pose.orientation.x = 0.0;
        goal_msg.poses[i].pose.orientation.y = 0.0;
        goal_msg.poses[i].pose.orientation.z = 0.7071068;
        goal_msg.poses[i].pose.orientation.w = 0.7071068;
    }
}

void GoToSiloPose::goal_abort_callback()
{
    if (this->x_horiz_line_detected || odom_msg.pose.pose.position.y <= -3.20)
    {
        if (this->is_on_line)
        {
            cancel_goal();
            this->done_flag = true;
            this->is_on_line = false;
            this->x_horiz_line_detected = false;
            RCLCPP_INFO(node_ptr_->get_logger(), " GoToSiloPose::Goal canceled");
        }
    }
}

namespace BT
{
    template <>
    inline uint8_t convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, '\n');
        if (parts.size() != 1)
        {
            throw RuntimeError("GoToSiloPose::invalid input)");
        }
        else
        {
            uint8_t output;
            output = convertFromString<int>(parts[0]);
            return output;
        }
    }
}
