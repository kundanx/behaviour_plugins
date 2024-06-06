#include "GoToSiloPose.hpp"   
#include <math.h>

/**********************************************************************************************************************
 * @brief ActionClient BT node to navigate to silo 
 * @brief Subscribed topics : silo_number [silo to navigate to] 
 *                          : junction_type [Abort navigation when landmark is reached and silo align action is called]
************************************************************************************************************************/

GoToSiloPose::GoToSiloPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    subscription_silonumber = node_ptr_->create_subscription<std_msgs::msg::UInt8>( 
        "/silo_number",
        qos_profile,
        std::bind(&GoToSiloPose::silo_subscriber_callback,this,std::placeholders::_1)
    );
    subscription_junctiontype = node_ptr_->create_subscription<std_msgs::msg::UInt8>( 
        "/junction_type",
        10,
        std::bind(&GoToSiloPose::junction_subscriber_callback,this,std::placeholders::_1)
    );
    subscription_isOnLine = node_ptr_->create_subscription<std_msgs::msg::UInt8>( 
        "is_on_line",
        10,
        std::bind(&GoToSiloPose::line_subscriber_callback,this,std::placeholders::_1)
    );
    done_flag = false;
    this->silo_number = 0;
    this->x_horiz_line_detected = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose node Ready..");

}

void GoToSiloPose::silo_subscriber_callback(std_msgs::msg::UInt8 msg)
{
    // RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose callback..");   
    this->silo_number = msg.data;
}
void GoToSiloPose::junction_subscriber_callback(std_msgs::msg::UInt8 msg)
{
    // RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose callback..");  
    if (msg.data == X_HORIZONTAL_LINE) 
    {
        this->x_horiz_line_detected = true;
    }
}
void GoToSiloPose::line_subscriber_callback(std_msgs::msg::UInt8 msg)
{
    this->is_on_line = msg.data;
}

BT::PortsList GoToSiloPose::providedPorts()
{
    return {BT::OutputPort<uint8_t>("Op_SiloNumber")};
}

BT::NodeStatus GoToSiloPose::onStart()
{
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    switch(this->silo_number)
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
            RCLCPP_INFO(node_ptr_->get_logger(),"silo number not recieved..[Default to zero]");
            return BT::NodeStatus::RUNNING;
    }

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&GoToSiloPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&GoToSiloPose::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GoToSiloPose::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    // make pose
    auto goal_msg = NavigateToPose::Goal();
    
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = this->pose[0];
    goal_msg.pose.pose.position.y = this->pose[1];
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.7071068;
    goal_msg.pose.pose.orientation.w = 0.7071068;

    // send pose
    y_coordinate = fabs(this->pose[1]);
    done_flag = false;
    x_horiz_line_detected = false;
    auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");

    // update silo_number
    setOutput<uint8_t>("Op_SiloNumber", silo_number);
    
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToSiloPose::onRunning()
{     
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToSiloPose::onHalted() 
{
     RCLCPP_WARN(node_ptr_->get_logger(),"Navigation aborted");
}

void GoToSiloPose::goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle)
  {
    if (!goal_handle) {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Goal was rejected by server");
    } 
      else
    {
          RCLCPP_INFO(node_ptr_->get_logger(), "Goal accepted by server, waiting for result");
        this->goal_handle = goal_handle;
    }
  }

void GoToSiloPose::result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}


/*****************************************************************************************************************
 * @brief Received feedback from server
 * @brief Abort navigation when robot crosses X_HORZ line and y-pose threashold
******************************************************************************************************************/

void GoToSiloPose::feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    if(this->x_horiz_line_detected ) //|| feedback->current_pose.pose.position.y >= -2.50
    {
        if(is_on_line)
        {
            cancel_goal();
            this->done_flag = true;
            this->is_on_line = false;
            this->x_horiz_line_detected = false;
        }
    }
    RCLCPP_INFO(node_ptr_->get_logger()," Navigating to silo %i",this->silo_number);
}

/*****************************************************************************************************************
 * @brief Cancle navigation action
******************************************************************************************************************/

 void GoToSiloPose::cancel_goal()
  {
     if (goal_handle) 
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Sending cancel request");
      auto cancel_future= action_client_ptr_->async_cancel_all_goals();
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    } 
    else 
    {
      RCLCPP_WARN(node_ptr_->get_logger(), "No active goal to cancel");
    } 
  }


