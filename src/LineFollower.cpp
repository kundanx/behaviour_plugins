#include "LineFollower.hpp"   
/*****************************************************************************************************************
 * @brief ActionClient BT node to call Linerfollower action server
 * @brief Available actions : Naviagte to area 3 
 *                          : Align with silo
 * @brief Published topics : area_topic [For landmark localization and odometry reset]
******************************************************************************************************************/


LineFollower::LineFollower(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    area3_reached_publisher = node_ptr_->create_publisher<std_msgs::msg::UInt8>("area_reached", qos_profile);
    action_client_ptr_ = rclcpp_action::create_client<LineFollow>(node_ptr_, "LineFollower");
    RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::Ready");
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
      RCLCPP_ERROR(node_ptr_->get_logger(), "LineFollower::Action server not available after waiting");
    }

    // Setup action client goal
    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<LineFollow>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LineFollower::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&LineFollower::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&LineFollower::result_callback, this, _1);
    
    auto goal_msg = LineFollow::Goal();
    done_flag = false;
    switch (action_type)
    {
        case NAVIGATE_FROM_START_ZONE:
            goal_msg.task = goal_msg.NAVIGATE_FROM_START_ZONE;
            break;

        case NAVIGATE_FROM_RETRY_ZONE:
            RCLCPP_WARN(node_ptr_->get_logger(), "LineFollower::Under maintenance");
            break;

        case ALIGN_W_SILO:
            goal_msg.task = goal_msg.ALIGN_W_SILO;
            break;

        case ALIGN_YAW:
            goal_msg.task = goal_msg.ALIGN_YAW;
            break;

        default :
            RCLCPP_ERROR(node_ptr_->get_logger(), "LineFollower::Invalid Action_type");
            return BT::NodeStatus::FAILURE;
    }
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(), "LineFollower::goal sent");
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
      auto cancel_future= action_client_ptr_->async_cancel_all_goals();
      RCLCPP_INFO(node_ptr_->get_logger(), "LineFollower::Goal canceled");
    } 
    else 
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "LineFollower::No active goal to cancel");
    } 
    rclcpp::shutdown();
  }

void LineFollower::goal_response_callback(const GoalHandleLineFollow::SharedPtr & goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "LineFollower::Goal rejected by server");
    } else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "LineFollower::Goal accepted by server, waiting for result");
        this->goal_handle = goal_handle;
    }
}

void LineFollower::feedback_callback(
    GoalHandleLineFollow::SharedPtr,
    const std::shared_ptr<const LineFollow::Feedback> feedback)
{
    (void)feedback;
}

/*****************************************************************************************************************
 * @brief Publish on area_topic once when naviagtion from start zone to area 3 is finished
******************************************************************************************************************/
void LineFollower::result_callback(const GoalHandleLineFollow::WrappedResult & wrappedresult)
{
    if(wrappedresult.result->robot_state == wrappedresult.result->NAVIGATION_START_ZONE_FINISHED)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower:: From start zone to Area 3 reached");
        std_msgs::msg::UInt8 msg;
        msg.data = 0xA5;
        area3_reached_publisher->publish(msg); 
        done_flag = true;
    }
    else if(wrappedresult.result->robot_state == wrappedresult.result->NAVIGATION_RETRY_ZONE_FINISHED)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower:: From retry zone to Area 3 reached");
        std_msgs::msg::UInt8 msg;
        msg.data = 0xA5;
        area3_reached_publisher->publish(msg); 
        done_flag = true;
    }
    else if (wrappedresult.result->robot_state == wrappedresult.result->ALIGNED_W_SILO )
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::Aligned With silo");
        done_flag = true;
    }
    else if (wrappedresult.result->robot_state == wrappedresult.result->ALIGNED_YAW )
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::YAW Aligned");
        done_flag = true;
    }
   
}
