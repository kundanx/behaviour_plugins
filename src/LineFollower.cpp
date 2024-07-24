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

    silo_number.data.resize(2);
    silo_number.data[0] = 0;
    silo_number.data[1] = 0;

    RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::Ready");
    done_flag = false;
}

BT::PortsList LineFollower::providedPorts()
{
    return {BT::InputPort<int>("Ip_action_type"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("In_pose"),
            BT::InputPort<std_msgs::msg::UInt8MultiArray>("Ip_SiloNumber"),
            BT::OutputPort<uint8_t>("Op_SiloNumber")};
}

BT::NodeStatus LineFollower::onStart()
{
    double goal_yaw = 1.57;
    auto action_type_ = getInput<int>("Ip_action_type");
    auto goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>("In_pose");
    auto silo_number_ = getInput<std_msgs::msg::UInt8MultiArray>("Ip_SiloNumber");

    if( !action_type_ )
    {
        throw BT::RuntimeError("error reading port [In_action_type]:", action_type_.error());
    }
    auto action_type = action_type_.value();

    if( goal_pose_)
    {
        auto goal_pose = goal_pose_.value();
        EulerAngles e;
        e = ToEulerAngles(goal_pose.pose.orientation.w,
                          goal_pose.pose.orientation.x,
                          goal_pose.pose.orientation.y,
                          goal_pose.pose.orientation.z);
        goal_yaw = e.yaw;
    }
    if ( silo_number_ )
    {
        silo_number = silo_number_.value();
    }
       
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
    
    LineFollow::Goal goal_msg;
    goal_msg.silo_numbers.data.resize(2);
    switch (action_type)
    {
        case NAVIGATE_FROM_START_ZONE:
            goal_msg.task = goal_msg.NAVIGATE_FROM_START_ZONE;
            goal_msg.rotate_to_angle = 0;
            goal_msg.silo_numbers.data[0] = 0;
            goal_msg.silo_numbers.data[1] = 0;
            break;

        case NAVIGATE_FROM_RETRY_ZONE:
            goal_msg.task = goal_msg.NAVIGATE_FROM_RETRY_ZONE;
            goal_msg.rotate_to_angle = 0;
            goal_msg.silo_numbers.data[0] = 0;
            goal_msg.silo_numbers.data[1] = 0;
            break;

        case ALIGN_W_SILO:
            goal_msg.task = goal_msg.ALIGN_W_SILO;
            goal_msg.rotate_to_angle = 0;
            goal_msg.silo_numbers = silo_number;
            break;

        case ALIGN_YAW:
            goal_msg.task = goal_msg.ALIGN_YAW;
            goal_msg.rotate_to_angle = 0;
            goal_msg.silo_numbers.data[0] = 0;
            goal_msg.silo_numbers.data[1] = 0;

            break;
        case ROTATE_TO_BALL:
            goal_msg.task = goal_msg.ROTATE_TO_BALL;
            goal_msg.rotate_to_angle = goal_yaw;
            goal_msg.silo_numbers.data[0] = 0;
            goal_msg.silo_numbers.data[1] = 0;
            break;

        default :
            RCLCPP_ERROR(node_ptr_->get_logger(), "LineFollower::Invalid Action_type");
            return BT::NodeStatus::FAILURE;
    }
    done_flag = false;
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
    // (void);

    cancel_goal();    
}

void LineFollower::cancel_goal()
  {
    if (goal_handle) 
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "LineFollower::Goal canceled");
      try
        {
            auto cancel_future = action_client_ptr_->async_cancel_goal(goal_handle);
        }
        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
        {
            RCLCPP_WARN(node_ptr_->get_logger(),"rclcpp_action::exceptions::UnknownGoalHandleError");
        }
    } 
    else 
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "LineFollower::No active goal to cancel");
    } 
    // rclcpp::shutdown();
  }

void LineFollower::goal_response_callback(const GoalHandleLineFollow::SharedPtr & goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "LineFollower::Goal rejected by server");
    } else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "LineFollower::Goal accepted by server, waiting for result");
        this->goal_handle = goal_handle_;
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
        setOutput<uint8_t>("Op_SiloNumber", wrappedresult.result->aligned_silo_number);
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::Aligned silo number %i", wrappedresult.result->aligned_silo_number);

        done_flag = true;
    }
    else if (wrappedresult.result->robot_state == wrappedresult.result->ALIGNED_YAW )
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::YAW Aligned");
        done_flag = true;
    }
    else if (wrappedresult.result->robot_state == wrappedresult.result->ROTATED_TO_BALL )
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"LineFollower::Rotated to Ball");
        done_flag = true;
    }
   
}
