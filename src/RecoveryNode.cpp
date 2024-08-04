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
    in_middle = false;
    left_right = LeftOrRight::N0_CHANGE;
    recovery_state = RecoveryState::HALT;
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

    Quaternion q;
    q = ToQuaternion(0.0, 0.0,  (1.57 * team_color));

    if ( fabs(odom_msg.pose.pose.position.y - (-0.50 * team_color)) <= 0.1)
    {
        in_middle = true;
    }

    if( !in_middle)
    {
        // send goal
        if ( fabs(odom_msg.pose.pose.position.y) >= 0.30)
        {
            recovery_state = RecoveryState::NAV;
        }
        else 
        {
            recovery_state = RecoveryState::BACKUP;

        }
        left_right = LeftOrRight::N0_CHANGE;

    }
    else
    {
        recovery_state = RecoveryState::HALT;
        if ( fabs(odom_msg.pose.pose.position.x - 0.0) < 0.3 )
        {
            if ( odom_msg.pose.pose.position.x < 0.0)
            {
                left_right = LeftOrRight::RIGHT;   
            }
            else
            {
                left_right = LeftOrRight::LEFT;

            }
        }
        else if ( fabs(odom_msg.pose.pose.position.x - 1.2) < 0.11)
        {
            left_right = RIGHT;   
        }
        else if ( fabs(odom_msg.pose.pose.position.x + 1.2) < 0.11)
        {
            left_right = LEFT;
        }
        else
        {
            left_right = LeftOrRight::N0_CHANGE;
            recovery_state = RecoveryState::NAV;
        }
       
    }

    switch (left_right)
    {
        case LeftOrRight::LEFT:
        {
            auto left_goal_msg = NavigateToPose::Goal();
            
            left_goal_msg.pose.header.frame_id = "map";
            left_goal_msg.pose.pose.position.x = 1.2;
            left_goal_msg.pose.pose.position.y = -0.50 * team_color;
            left_goal_msg.pose.pose.position.z = 0.0;
        
            left_goal_msg.pose.pose.orientation.x = q.x;
            left_goal_msg.pose.pose.orientation.y = q.y;
            left_goal_msg.pose.pose.orientation.z = q.z;
            left_goal_msg.pose.pose.orientation.w = q.w;

            nav_action_client_ptr_->async_send_goal(left_goal_msg, nav_send_goal_options);   
            RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::nav to left sent");

            break;
        }

        case LeftOrRight::RIGHT:
        {
            auto right_goal_msg = NavigateToPose::Goal();

            right_goal_msg.pose.header.frame_id = "map";
            right_goal_msg.pose.pose.position.x = -1.2;
            right_goal_msg.pose.pose.position.y = -0.50 * team_color;
            right_goal_msg.pose.pose.position.z = 0.0;

            right_goal_msg.pose.pose.orientation.x = q.x;
            right_goal_msg.pose.pose.orientation.y = q.y;
            right_goal_msg.pose.pose.orientation.z = q.z;
            right_goal_msg.pose.pose.orientation.w = q.w;

            nav_action_client_ptr_->async_send_goal(right_goal_msg, nav_send_goal_options); 
            RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::nav to right sent");

            break;       
        }
            
    }

    switch (recovery_state)
    {
        case RecoveryState::NAV:
        {
            auto middle_goal_msg = NavigateToPose::Goal();
        
            middle_goal_msg.pose.header.frame_id = "map";
            middle_goal_msg.pose.pose.position.x = 0.0;
            middle_goal_msg.pose.pose.position.y = -0.50 * team_color;
            middle_goal_msg.pose.pose.position.z = 0.0;
        
            middle_goal_msg.pose.pose.orientation.x = q.x;
            middle_goal_msg.pose.pose.orientation.y = q.y;
            middle_goal_msg.pose.pose.orientation.z = q.z;
            middle_goal_msg.pose.pose.orientation.w = q.w;
            nav_action_client_ptr_->async_send_goal(middle_goal_msg, nav_send_goal_options); 
            RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::nav to middle sent");

            break;
        }

        case RecoveryState::BACKUP:
        {
            auto goal_back_up = LineFollow::Goal();
            goal_back_up.silo_numbers.data.resize(2);
            goal_back_up.task = goal_back_up.BACK_UP;
            goal_back_up.back_dist = 0.3;
            goal_back_up.rotate_to_angle = 0;
            goal_back_up.silo_numbers.data[0] = 0;
            goal_back_up.silo_numbers.data[1] = 0;
            backUp_action_client_ptr_->async_send_goal(goal_back_up, backUp_send_goal_options);  
            RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::Back up sent");

            break;       
        }
    }

    if ( (recovery_state == RecoveryState::HALT) && (left_right == LeftOrRight::N0_CHANGE))
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::HALT state.. No goal sent");
        return BT::NodeStatus::SUCCESS;
    }

    start_time = get_tick_ms();
    prev_x = odom_msg.pose.pose.position.x;
    prev_y = odom_msg.pose.pose.position.y;

    done_flag = false;
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RecoveryNode::onRunning()
{   
    // if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.01 && fabs((prev_y - odom_msg.pose.pose.position.y)) < 0.01)
    // {
    //     uint32_t now = get_tick_ms();
    //     if( now - start_time >= 5000)
    //     {
    //         cancel_goal();
    //         this->done_flag = true;
    //         RCLCPP_INFO(node_ptr_->get_logger(), " RecoveryNode::Robot static.. cancel goal ");
    //         return BT::NodeStatus::SUCCESS; 

    //     }   
    //     else
    //     {
    //         start_time = get_tick_ms();
    //     }
    // }
    // else
    // {
    //     prev_x = odom_msg.pose.pose.position.x;
    //     prev_y = odom_msg.pose.pose.position.y;
    //     start_time = get_tick_ms();
    // }

    uint32_t now = get_tick_ms();
    if ( now - start_time >= 3000)
    {
        if( fabs(prev_x - odom_msg.pose.pose.position.x) < 0.01 && fabs((prev_y - odom_msg.pose.pose.position.y)) < 0.01)
        {
            cancel_goal();
            this->done_flag = true;
            RCLCPP_INFO(node_ptr_->get_logger(), " RecoveryNode::Robot static.. cancel goal ");
            return BT::NodeStatus::SUCCESS; 

        }
        else
        {
            start_time = get_tick_ms();
        }
    }

    if(done_flag)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void RecoveryNode::onHalted() 
{
   cancel_goal();
}

void RecoveryNode::cancel_goal()
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
        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Nav goal accepted ");
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

