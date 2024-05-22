#include "LineFollower.hpp"   

LineFollower::LineFollower(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<LineFollow>(node_ptr_, "LineFollower");
    RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower node Ready..");
    done_flag = false;
}

BT::PortsList LineFollower::providedPorts()
{
    return {BT::InputPort<int>("Ip_action_type")};
}

BT::NodeStatus LineFollower::onStart()
{
    auto action_type_ = getInput<int>("Ip_action_type");
    if( !action_type_ )
    {
        throw BT::RuntimeError("error reading port [In_action_type]:", action_type_.error());
    }
    auto action_type = action_type_.value();

    // make pose
    if (!this->action_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available after waiting");
    }

    // Setup action client goal
    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<LineFollow>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LineFollower::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&LineFollower::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&LineFollower::result_callback, this, _1);
    
    auto goal_msg = LineFollow::Goal();
    switch (action_type)
    {
        case NAVIGATE:
            goal_msg.task = goal_msg.NAVIGATE;
            break;

        case ALIGN_W_SILO:
            goal_msg.task = goal_msg.ALIGN_W_SILO;
            break;

        case ALIGN_W_ORIGIN:
            goal_msg.task = goal_msg.ALIGN_W_ORIGIN;
            break;

        default :
            RCLCPP_ERROR(node_ptr_->get_logger(), "Invalid Action_type");
            return BT::NodeStatus::FAILURE;
    }
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(), "goal sent");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus LineFollower::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void LineFollower::onHalted() 
{
    cancel_goal();    
}

void LineFollower::cancel_goal()
  {
    if (goal_handle) 
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Sending cancel request");
      auto cancel_future= action_client_ptr_->async_cancel_goal(goal_handle);
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    } 
    else 
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "No active goal to cancel");
    } 
    rclcpp::shutdown();
  }

void LineFollower::goal_response_callback(const GoalHandleLineFollow::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Goal accepted by server, waiting for result");
    }
    this->goal_handle = goal_handle;
}

void LineFollower::feedback_callback(
    GoalHandleLineFollow::SharedPtr,
    const std::shared_ptr<const LineFollow::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(node_ptr_->get_logger(),"Executing line follower task");
}

void LineFollower::result_callback(const GoalHandleLineFollow::WrappedResult & result)
{
    (void)result;
    RCLCPP_INFO(node_ptr_->get_logger(),"Goal Reacched");
    done_flag = true;
}
