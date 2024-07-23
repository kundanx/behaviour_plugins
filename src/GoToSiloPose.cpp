#include "GoToSiloPose.hpp"
#include <math.h>

using namespace std::chrono_literals;

/**********************************************************************************************************************
 * @brief ActionClient BT node to navigate to silo
 * @brief Subscribed topics : silo_number [silo to navigate to]
 *                          : junction_type [Abort navigation when landmark is reached and silo align action is called]
 ************************************************************************************************************************/
int silo_check;
GoToSiloPose::GoToSiloPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");

    color_feedback_publisher = node_ptr_->create_publisher<std_msgs::msg::Int8>("color_feedback/GoToSiloPose", qos_profile);

    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        qos_profile,
        std::bind(&GoToSiloPose::odometry_callback, this, std::placeholders::_1));
    subscription_silonumber = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(
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
    subscription_team_color = node_ptr_->create_subscription<std_msgs::msg::Int8>(
        "team_color",
        qos_profile,
        std::bind(&GoToSiloPose::team_color_callback, this, std::placeholders::_1));

    done_flag = false;
    this->x_horiz_line_detected = false;
    silo_numbers.data.resize(2);
    silo_check = 5;

    RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Ready");
}

void GoToSiloPose::silo_subscriber_callback(const std_msgs::msg::UInt8MultiArray &msg)
{
    this->silo_numbers = msg;
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
    this->is_on_line = msg.data;
}

void GoToSiloPose::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    this->odom_msg = msg;
}

void GoToSiloPose::team_color_callback(const std_msgs::msg::Int8 &msg)
{
    if( msg.data == -1)
        team_color = RED;
    else
        team_color = BLUE;
    color_feedback_publisher->publish(msg);
    
}

BT::PortsList GoToSiloPose::providedPorts()
{
    return {BT::InputPort<int>("Ip_SiloNumber"),
            BT::OutputPort<std_msgs::msg::UInt8MultiArray>("Op_SiloNumber"),
            BT::InputPort<int>("In_start_wait")};
}

BT::NodeStatus GoToSiloPose::onStart()
{

    auto silo_number_ = getInput<int>("Ip_SiloNumber");
   
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    this->testing = locations["testing"].as<bool>();   
    if (silo_number_)
    {
        if (testing)
        {
            this->silo_numbers.data[0] = silo_number_.value();
        }
    }
    if( silo_check == 5)
    {
        this->silo_numbers.data[0] = 1;
        this->silo_numbers.data[1] = 2;
        silo_check = 1;
    }
    else if ( silo_check == 1)
    {
        this->silo_numbers.data[0] = 2;
        this->silo_numbers.data[1] = 3;
        silo_check = 2;

    }
    else if ( silo_check == 2)
    {
        this->silo_numbers.data[0] = 3;
        this->silo_numbers.data[1] = 4;
        silo_check = 3;

    }
    else if ( silo_check == 3)
    {
        this->silo_numbers.data[0] = 4;
        this->silo_numbers.data[1] = 5;
        silo_check = 4;

    }
    else if ( silo_check == 4)
    {
        this->silo_numbers.data[0] = 5;
        this->silo_numbers.data[1] = 1;
        silo_check = 5;

    }
    else
    {
        silo_check = 5;   
    }

    if(compute_goal_NavTo() == -1)
    { 
        return BT::NodeStatus::FAILURE;
    }
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GoToSiloPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToSiloPose::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToSiloPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    done_flag = false;
    x_horiz_line_detected = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    setOutput<std_msgs::msg::UInt8MultiArray>("Op_SiloNumber", silo_numbers);
    RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::sent goal");

    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToSiloPose::onRunning()
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

    if ( odom_msg.pose.pose.position.y <= -2.20 && fabs(odom_msg.pose.pose.position.x -  goal_msg.pose.pose.position.x) < 0.3 )
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
    cancel_goal();
}

void GoToSiloPose::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "GoToSiloPose::Navigate to silo %i goal rejected by server", this->silo_numbers.data[0]);
        setOutput<uint8_t>("Op_SiloNumber", 0);
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::Navigating to silo %i ", this->silo_numbers.data[0]);
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

void GoToSiloPose::feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;

}

void GoToSiloPose::cancel_goal()
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

int GoToSiloPose::compute_goal_NavTo()
{

    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    YAML::Node locations = YAML::LoadFile(location_file);
    std::vector<float> updated_pose{2};


    switch (this->silo_numbers.data[0])
    {
    case 1:
        pose = locations["silo_one"].as<std::vector<float>>();
        break;
    case 2:
        pose = locations["silo_two"].as<std::vector<float>>();
        break;
    case 3:
        pose = locations["silo_three"].as<std::vector<float>>();
        break;
    case 4:
        pose = locations["silo_four"].as<std::vector<float>>();
        break;
    case 5:
        pose = locations["silo_five"].as<std::vector<float>>();
        break;

    default:
        RCLCPP_INFO(node_ptr_->get_logger(), "GoToSiloPose::silo number not recieved..[Default to zero]");
        return -1;
    }

    Quaternion q;
    q = ToQuaternion(0.0, 0.0,  (1.57 * team_color));

    
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1] * team_color;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = q.x;
    goal_msg.pose.pose.orientation.y = q.y;
    goal_msg.pose.pose.orientation.z = q.z;
    goal_msg.pose.pose.orientation.w = q.w;
    return 0;

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
