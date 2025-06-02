#include "BackUpActionClient.hpp"   

/*****************************************************************************************************************
 * @brief ActionClient BT node to call nav2_behaior spin action
******************************************************************************************************************/

BackUpActionClient::BackUpActionClient(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<BackUp>(node_ptr_, "/backup");
    RCLCPP_INFO(node_ptr_->get_logger(),"BackUpActionClient node Ready..");
    done_flag = false;
}

BT::PortsList BackUpActionClient::providedPorts()
{
    return {BT::InputPort<float64>("In_dist"),
        BT::InputPort<float32>("In_speed")
    };
}

BT::NodeStatus BackUpActionClient::onStart()
{
    auto input_data = getInput<float64>("In_dist");
    auto input_data_speed = getInput<float32>("In_speed");
    if( !input_data )
    {
        throw BT::RuntimeError("BackUpActionClient::error reading port [In_dist]:", input_data.error());
    }
     if( !input_data_speed )
    {
        throw BT::RuntimeError("BackUpActionClient::error reading port [In_speed]:", input_data.error());
    }
    
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&BackUpActionClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.result_callback = std::bind(&BackUpActionClient::backUp_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&BackUpActionClient::backUp_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make goal::target_yaw
    backUp_dist.target.x = input_data.value().data;
    backUp_dist.target.y = 0.0;
    backUp_dist.target.z = 0.0;

    backUp_dist.speed = input_data_speed.value().data;

    // send goal::target_yaw
    done_flag = false;
    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    action_client_ptr_->async_send_goal(backUp_dist, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to BackUpActionServer [%f] \n", backUp_dist.target.x);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUpActionClient::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] goal reached [: %f] \n", this->name().c_str(), backUp_dist.target.x);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void BackUpActionClient::onHalted() 
{
    cancel_goal();
    // auto cancel_future= action_client_ptr_->async_cancel_goal(goal_handle);
}

void BackUpActionClient::cancel_goal()
{
    if (goal_handle)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "BackUpActionClient::Goal canceled");
        try
        {
            auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        }
        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
        {
            RCLCPP_WARN(node_ptr_->get_logger(),"BackUpActionClient::cancel_goal::rclcpp_action::exceptions::UnknownGoalHandleError");
        }
    }
    else
    {
        RCLCPP_WARN(node_ptr_->get_logger(), "BackUpActionClient::No active goal to cancel");
    }
}

void BackUpActionClient::goal_response_callback(const GoalHandleBackUp::SharedPtr &goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "BackUpActionClient::goal rejected ");
    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "BackUpActionClient::Goal accepted ");
        this->goal_handle = goal_handle_;

    }

}

void BackUpActionClient::backUp_result_callback(const GoalHandleBackUp::WrappedResult &result)
{
    if(result.result)
        done_flag=true;
}

void BackUpActionClient::backUp_feedback_callback(
    GoalHandleBackUp::SharedPtr,
    const std::shared_ptr<const BackUp::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(node_ptr_->get_logger(),"[distance_travelled: %f]",feedback->distance_traveled);
}

namespace BT
{
    template <> inline std_msgs::msg::Float64 convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, '\n');
        if (parts.size() != 1)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            std_msgs::msg::Float64 output;
            output.data    = convertFromString<float>(parts[0]);
            return output;
        }
    }
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