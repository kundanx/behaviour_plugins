#include "waitActionClient.hpp"   

/*****************************************************************************************************************
 * @brief ActionCient BT node to call nav2_behavior wait action
******************************************************************************************************************/

waitActionClient::waitActionClient(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<Wait>(node_ptr_, "/wait");
    RCLCPP_INFO(node_ptr_->get_logger(),"waitActionClient node Ready..");
    done_flag = false;
}

BT::PortsList waitActionClient::providedPorts()
{
    return {
        BT::InputPort<int32>("In_sec", "wait secs"),
        BT::InputPort<uint32_t>("In_nanosec","wait nanosecs"),
    };
}

BT::NodeStatus waitActionClient::onStart()
{
    auto seconds = getInput<int32>("In_sec");
    auto nanoseconds = getInput<uint32_t>("In_nanosec");
    if( !seconds )
    {
        throw BT::RuntimeError("waitActionClient::error reading port [In_sec]:", seconds.error());
    }
    if( !nanoseconds)
    {
        throw BT::RuntimeError("waitActionClient::error reading port [In_nanosec]:", nanoseconds.error());
    }
    
    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<Wait>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&waitActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&waitActionClient::wait_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&waitActionClient::wait_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make goal::target_yaw
    wait.time.sec = seconds.value().data;
    wait.time.nanosec = nanoseconds.value();

    // send goal::target_yaw
    done_flag = false;
    // action_client_ptr_->async_send_goal(wait, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"waitActionClient::sent goal to waitActionServer [%i seconds]", wait.time.sec);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus waitActionClient::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] goal reached [waited: %i seconds] \n", this->name().c_str(), wait.time.sec);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void waitActionClient::onHalted() 
{
     RCLCPP_WARN(node_ptr_->get_logger(),"waitActionClient::Wait aborted");
}

void waitActionClient::goal_response_callback(const rclcpp_action::ClientGoalHandle<Wait>::SharedPtr & goal_handle_)
{
     if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "waitActionClient::Wait goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "waitActionClient::Waiting ");
        this->goal_handle = goal_handle_;
    }
}

void waitActionClient::wait_result_callback(const GoalHandleWait::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
        RCLCPP_WARN(node_ptr_->get_logger(),"waitActionClient::Wait complete");
        
    }
}

void waitActionClient::wait_feedback_callback(
    GoalHandleWait::SharedPtr,
    const std::shared_ptr<const Wait::Feedback> feedback)
{
    (void)feedback;
    // RCLCPP_INFO(node_ptr_->get_logger(),"waitActionClient::waiting]");
}

//  Implemented from documentation: Generic port
namespace BT
{
    template <> inline std_msgs::msg::Int32 convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, '\n');
        if (parts.size() != 1)
        {
            throw RuntimeError("waitActionClient::invalid input)");
        }
        else
        {
            std_msgs::msg::Int32 output;
            output.data    = convertFromString<float>(parts[0]);
            return output;
        }
    }
} // end names