#include "spinActionClient.hpp"   

/*****************************************************************************************************************
 * @brief ActionClient BT node to call nav2_behaior spin action
******************************************************************************************************************/

spinActionClient::spinActionClient(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    action_client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&spinActionClient::odometry_callback,this,std::placeholders::_1)
    );
    RCLCPP_INFO(node_ptr_->get_logger(),"spinActionClient node Ready..");
    done_flag = false;
}

BT::PortsList spinActionClient::providedPorts()
{
    return {BT::InputPort<float32>("In_angle")};
}

BT::NodeStatus spinActionClient::onStart()
{
    auto input_data = getInput<float32>("In_angle");
    if( !input_data )
    {
        throw BT::RuntimeError("error reading port [In_angle]:", input_data.error());
    }
    auto input_yaw =  input_data.value().data;

    // make goal::target_yaw
    goal_spin_yaw.target_yaw = input_yaw;
    
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&spinActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&spinActionClient::spin_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&spinActionClient::spin_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);


    // send goal::target_yaw
    done_flag = false;
    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    action_client_ptr_->async_send_goal(goal_spin_yaw, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to SpinActionServer [%f] \n", goal_spin_yaw.target_yaw);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus spinActionClient::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] goal reached [rotated: %f] \n", this->name().c_str(), goal_spin_yaw.target_yaw);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void spinActionClient::onHalted() 
{
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    RCLCPP_WARN(node_ptr_->get_logger(),"Rotation aborted");
}

void spinActionClient::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    odom_msg = msg;
}

void spinActionClient::goal_response_callback(const rclcpp_action::ClientGoalHandle<Spin>::SharedPtr & goal_handle_)
{
     if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "spinActionClient::Spin goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "waitActionClient::Spining ");
        this->goal_handle = goal_handle_;
    }
}


void spinActionClient::spin_result_callback(const GoalHandleSpin::WrappedResult &result)
{
    if(result.result)
        done_flag=true;
}

void spinActionClient::spin_feedback_callback(
    GoalHandleSpin::SharedPtr,
    const std::shared_ptr<const Spin::Feedback> feedback)
{
    (void)feedback;
    // RCLCPP_INFO(node_ptr_->get_logger(),"[angular_distance_travelled: %f]",feedback->angular_distance_traveled);
}

namespace BT
{
    template <> inline std_msgs::msg::Float32 convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, '\n');
        if (parts.size() != 1)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            std_msgs::msg::Float32 output;
            output.data    = convertFromString<float>(parts[0]);
            return output;
        }
    }
} // end names