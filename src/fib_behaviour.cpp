#include "fib_behaviour.h"

Fib::Fib(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<Fibonacci>(node_ptr_, "fibonacci");
    done_flag = false;
}
BT::PortsList Fib::providedPorts()
{
    return {BT::InputPort<std::string>("order")};
}

BT::NodeStatus Fib::onStart()
{
    BT::Expected<std::string> goal = getInput<std::string>("order");
    const std::string config_file = node_ptr_->get_parameter("location_file").as_string();

    YAML::Node config = YAML::LoadFile(config_file);
    int32_t order = config[goal.value()].as<int32_t>();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Fib::result_callback, this, std::placeholders::_1);

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = order;
    
    //send goal
    done_flag = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(), "Fib goal sent");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus Fib::onRunning()
{
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "Goal completed");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
void Fib::result_callback(const GoalHandleFibonacci::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
