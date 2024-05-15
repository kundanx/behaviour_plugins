#include "GoToSiloPose.hpp"   

GoToSiloPose::GoToSiloPose(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( 
        "/silo_number",
        10,
        std::bind(&GoToSiloPose::subscriber_callback,this,std::placeholders::_1)
    );
    done_flag = false;
    this->silo_number = 0;
    RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose node Ready..");

}

void GoToSiloPose::subscriber_callback(std_msgs::msg::UInt8 msg)
{
    // RCLCPP_INFO(node_ptr_->get_logger(),"GoToSiloPose callback..");   
    this->silo_number = msg.data;
}

BT::NodeStatus GoToSiloPose::onStart()
{
    this->goal_sent_flag = false;
    
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToSiloPose::onRunning()
{   
    if(!this->goal_sent_flag)
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
        send_goal_options.result_callback = std::bind(&GoToSiloPose::nav_to_pose_result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&GoToSiloPose::nav_to_pose_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

        // make pose
        auto goal_msg = NavigateToPose::Goal();
        
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = this->pose[0];
        goal_msg.pose.pose.position.y = this->pose[1];
        goal_msg.pose.pose.position.z = 0.0;

        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        // send pose
        done_flag = false;
        action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");
        this->goal_sent_flag = true;
    }
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        this->goal_sent_flag = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void GoToSiloPose::onHalted() 
{
     RCLCPP_WARN(node_ptr_->get_logger(),"Navigation aborted");
}

void GoToSiloPose::nav_to_pose_result_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}
void GoToSiloPose::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(node_ptr_->get_logger()," Navigating to silo %i",this->silo_number);
}


