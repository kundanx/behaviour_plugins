#include "spinActionClient.hpp"   

spinActionClient::spinActionClient(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
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
    
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&spinActionClient::spin_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&spinActionClient::spin_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make goal::target_yaw
    goal_spin_yaw.target_yaw = input_data.value().data;

    // send goal::target_yaw
    done_flag = false;
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
     RCLCPP_WARN(node_ptr_->get_logger(),"Rotation aborted");
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
    RCLCPP_INFO(node_ptr_->get_logger(),"[angular_distance_travelled: %f]",feedback->angular_distance_traveled);
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