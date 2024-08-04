#include "RecoveryNode.hpp"   

/*****************************************************************************************************************
 * @brief Tree Node to bring robot back to origin to intake new ball after ball is stored in silo
******************************************************************************************************************/

RecoveryNode::RecoveryNode(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    nav_action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    backUp_action_client_ptr_ = rclcpp_action::create_client<LineFollow>(node_ptr_, "LineFollower");

    color_feedback_publisher = node_ptr_->create_publisher<std_msgs::msg::Int8>("color_feedback/RecoveryNode", qos_profile);
    
    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&RecoveryNode::odometry_callback,this,std::placeholders::_1)
    );

    subscription_team_color = node_ptr_->create_subscription<std_msgs::msg::Int8>(
        "team_color",
        qos_profile,
        std::bind(&RecoveryNode::team_color_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::Ready");
    done_flag = false;
    recovery_state = HALT;
}

BT::NodeStatus RecoveryNode::onStart()
{
    auto nav_send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    nav_send_goal_options.goal_response_callback =std::bind(&RecoveryNode::nav_goal_response_callback, this, std::placeholders::_1);
    nav_send_goal_options.result_callback = std::bind(&RecoveryNode::nav_result_callback, this, std::placeholders::_1);
    nav_send_goal_options.feedback_callback = std::bind(&RecoveryNode::nav_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    auto backUp_send_goal_options = rclcpp_action::Client<LineFollow>::SendGoalOptions();
    backUp_send_goal_options.goal_response_callback =std::bind(&RecoveryNode::backUp_goal_response_callback, this, std::placeholders::_1);
    backUp_send_goal_options.result_callback = std::bind(&RecoveryNode::backUp_result_callback, this, std::placeholders::_1);
    backUp_send_goal_options.feedback_callback = std::bind(&RecoveryNode::backUp_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

   
    // make nav goal pose
    auto nav_goal_msg = NavigateToPose::Goal();
    Quaternion q;
    q = ToQuaternion(0.0, 0.0,  (1.57 * team_color));
        
    nav_goal_msg.pose.header.frame_id = "map";
    nav_goal_msg.pose.pose.position.x = 0.0;
    nav_goal_msg.pose.pose.position.y = -0.40 * team_color;
    nav_goal_msg.pose.pose.position.z = 0.0;

    nav_goal_msg.pose.pose.orientation.x = q.x;
    nav_goal_msg.pose.pose.orientation.y = q.y;
    nav_goal_msg.pose.pose.orientation.z = q.z;
    nav_goal_msg.pose.pose.orientation.w = q.w;

  

    auto goal_back_up = LineFollow::Goal();
    goal_back_up.silo_numbers.data.resize(2);
    goal_back_up.task = goal_back_up.BACK_UP;
    goal_back_up.back_dist = 0.3;
    goal_back_up.rotate_to_angle = 0;
    goal_back_up.silo_numbers.data[0] = 0;
    goal_back_up.silo_numbers.data[1] = 0;

    // send goal
    if ( fabs(odom_msg.pose.pose.position.y) >= 1.55)
    {
        recovery_state = RecoveryState::NAV;
    }
    else 
    {
        recovery_state = RecoveryState::BACKUP;

    }

    switch (recovery_state)
    {
        case NAV:
            nav_action_client_ptr_->async_send_goal(nav_goal_msg, nav_send_goal_options);   
            break;

        case BACKUP:
            backUp_action_client_ptr_->async_send_goal(goal_back_up, backUp_send_goal_options);  
            break;       
            
        default:    
            return BT::NodeStatus::SUCCESS;
    }

    done_flag = false;
    RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::Action sent");

    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RecoveryNode::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode::Goal reached");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void RecoveryNode::onHalted() 
{
    switch (recovery_state)
    {
        case RecoveryState::NAV:
            if (nav_goal_handle)
            {
                try
                    {
                        auto cancel_future = nav_action_client_ptr_->async_cancel_goal(nav_goal_handle);
                    }
                    catch(rclcpp_action::exceptions::UnknownGoalHandleError)
                    {
                        RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::onHalted::NAV::rclcpp_action::exceptions::UnknownGoalHandleError");
                    }
                    RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Nav Goal canceled");
            }
            break;

        case RecoveryState::BACKUP:
            if (backUp_goal_handle)
                {
                    try
                        {
                            auto cancel_future = backUp_action_client_ptr_->async_cancel_goal(backUp_goal_handle);
                        }
                        catch(rclcpp_action::exceptions::UnknownGoalHandleError)
                        {
                            RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::onHalted::BACKUP::rclcpp_action::exceptions::UnknownGoalHandleError");
                        }
                        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::BackUp Goal canceled");
                }
            break;
        
        default:
            RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::onHalted:: No goal to cancel");
            return ;
    }
  
}

void RecoveryNode::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    odom_msg = msg;
}

void RecoveryNode::team_color_callback(const std_msgs::msg::Int8 &msg)
{
    if( msg.data == -1)
        team_color = RED;
    else
        team_color = BLUE;
        
    color_feedback_publisher->publish(msg);
    
}

void RecoveryNode::nav_result_callback(const GoalHandleNav::WrappedResult &wrappedresult)
{
    if(wrappedresult.result)
    {
        done_flag=true;
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: Nav complete");

    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: Nav failed");

    }
}
void RecoveryNode::nav_feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}

void RecoveryNode::nav_goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & goal_handle_)
{
    if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "RecoveryNode::Nav recovery rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Nav to Top ");
        this->nav_goal_handle = goal_handle_;
    }
}

void RecoveryNode::backUp_result_callback(const GoalHandleLineFollow::WrappedResult &wrappedresult)
{
    if(wrappedresult.result)
    {
        done_flag=true;
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: Backup Complete");

    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: BackUp failed");

    }
}

void RecoveryNode::backUp_feedback_callback(
    GoalHandleLineFollow::SharedPtr,
    const std::shared_ptr<const LineFollow::Feedback> feedback)
{
    (void)feedback;
}

void RecoveryNode::backUp_goal_response_callback(const rclcpp_action::ClientGoalHandle<LineFollow>::SharedPtr & goal_handle_)
{
    if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "RecoveryNode::Back up goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Backing Up  ");
        this->backUp_goal_handle = goal_handle_;
    }
}

