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
    backUp_action_client_ptr_ = rclcpp_action::create_client<BackUp>(node_ptr_, "/backup");
    spin_action_client_ptr_ = rclcpp_action::create_client<Spin>(node_ptr_, "/spin");
    align_yaw_action_client_ptr_ = rclcpp_action::create_client<LineFollow>(node_ptr_, "/LineFollower");


    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&RecoveryNode::odometry_callback,this,std::placeholders::_1)
    );

    subscription_ballpose = node_ptr_->create_subscription<oakd_msgs::msg::StatePose>( 
        "/ball_tracker", 
        qos_profile, 
        std::bind(&RecoveryNode::ballpose_callback,this,std::placeholders::_1)
    );

    RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::Ready");
    done_flag = false;
    recovery_state = HALT;
    spin_counter = MAX_SPIN_NUM;
    // RecoveryState recoveryType = HALT;
}

BT::PortsList RecoveryNode::providedPorts()
{
    return{BT::InputPort<int>("Ip_team_color")};
}


BT::NodeStatus RecoveryNode::onStart()
{
    auto team_color_ = getInput<int>("Ip_team_color");
    if(team_color_)
    {
         if (team_color_.value() == -1)
        {
            team_color = RED;
        }
        else 
            team_color = BLUE;
    }
    else 
        team_color = RED;


   

    auto nav_send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    nav_send_goal_options.goal_response_callback =std::bind(&RecoveryNode::nav_goal_response_callback, this, std::placeholders::_1);
    nav_send_goal_options.result_callback = std::bind(&RecoveryNode::nav_result_callback, this, std::placeholders::_1);
    nav_send_goal_options.feedback_callback = std::bind(&RecoveryNode::nav_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    auto backUp_send_goal_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
    backUp_send_goal_options.goal_response_callback =std::bind(&RecoveryNode::backUp_goal_response_callback, this, std::placeholders::_1);
    backUp_send_goal_options.result_callback = std::bind(&RecoveryNode::backUp_result_callback, this, std::placeholders::_1);
    backUp_send_goal_options.feedback_callback = std::bind(&RecoveryNode::backUp_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    auto spin_send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
    spin_send_goal_options.goal_response_callback =std::bind(&RecoveryNode::spin_goal_response_callback, this, std::placeholders::_1);
    spin_send_goal_options.result_callback = std::bind(&RecoveryNode::spin_result_callback, this, std::placeholders::_1);
    spin_send_goal_options.feedback_callback = std::bind(&RecoveryNode::spin_feedback_callback, this, std::placeholders::_1,std::placeholders::_2);

    auto align_yaw_send_goal_options = rclcpp_action::Client<LineFollow>::SendGoalOptions();
    align_yaw_send_goal_options.goal_response_callback = std::bind(&RecoveryNode::align_yaw_goal_response_callback, this,std::placeholders::_1);
    align_yaw_send_goal_options.feedback_callback = std::bind(&RecoveryNode::align_yaw_feedback_callback, this,std::placeholders::_1,std::placeholders::_2);
    align_yaw_send_goal_options.result_callback = std::bind(&RecoveryNode::align_yaw_result_callback, this,std::placeholders::_1);
    


    // make nav goal pose
    auto goal_msg = NavigateToPose::Goal();
    Quaternion q;
    q = ToQuaternion(0.0, 0.0,  (1.57 * team_color));
        
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = -2.0;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = q.x;
    goal_msg.pose.pose.orientation.y = q.y;
    goal_msg.pose.pose.orientation.z = q.z;
    goal_msg.pose.pose.orientation.w = q.w;

    // make backUp goal distance
    auto  backUp_dist = BackUp::Goal();

    backUp_dist.target.x = 0.3;
    backUp_dist.target.y = 0.0;
    backUp_dist.target.z = 0.0;
    backUp_dist.speed = 1.0;

    auto goal_spin = Spin::Goal();
    goal_spin.target_yaw = 15;

    auto goal_align = LineFollow::Goal();
    goal_align.silo_numbers.data.resize(2);
    goal_align.task = goal_align.ALIGN_YAW;
    goal_align.rotate_to_angle = 0;
    goal_align.silo_numbers.data[0] = 0;
    goal_align.silo_numbers.data[1] = 0;

    // send goal
    if ( odom_msg.pose.pose.position.y <= -1.55)
    {
        recovery_state = RecoveryState::NAV;
    }
    // else if ( ball_drift != NO_DRIFT )
    // {
    //     if (spin_counter < MAX_SPIN_NUM)
    //     {
    //         recovery_state = RecoveryState::SPIN;
    //         spin_counter++;
    //     }
    //     else
    //     {
    //         ball_drift = NO_DRIFT;
    //         recovery_state = RecoveryState::YAW_ALIGN;
    //     }
    // }
    else 
    {
        recovery_state = RecoveryState::BACKUP;

        // if ( backup_counter < MAX_BACKUP_NUM)
        // {
            // recovery_state = RecoveryState::BACKUP;
        // }    
    }

    switch (recovery_state)
    {
        case NAV:
            nav_action_client_ptr_->async_send_goal(goal_msg, nav_send_goal_options);   
            break;

        case SPIN:
            spin_action_client_ptr_->async_send_goal(goal_spin, spin_send_goal_options);
            break;
        
        case BACKUP:
            backUp_action_client_ptr_->async_send_goal(backUp_dist, backUp_send_goal_options);    
            break;

        case YAW_ALIGN:
            recovery_state = RecoveryState::BACKUP;
            align_yaw_action_client_ptr_->async_send_goal(goal_align, align_yaw_send_goal_options);  
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
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void RecoveryNode::onHalted() 
{
    switch (recovery_state)
    {
        case NAV:
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

        case SPIN:
         if (spin_goal_handle)
            {
                try
                    {
                        auto cancel_future = spin_action_client_ptr_->async_cancel_goal(spin_goal_handle);
                    }
                    catch(rclcpp_action::exceptions::UnknownGoalHandleError)
                    {
                        RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::onHalted::SPIN::rclcpp_action::exceptions::UnknownGoalHandleError");
                    }
                    RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Spin Goal canceled");
            }
            break;

        case BACKUP:
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
        
        case YAW_ALIGN:
         if (align_yaw_goal_handle) 
            {
                try
                    {
                        auto cancel_future = align_yaw_action_client_ptr_->async_cancel_goal(align_yaw_goal_handle);
                    }
                    catch(rclcpp_action::exceptions::UnknownGoalHandleError)
                    {
                        RCLCPP_WARN(node_ptr_->get_logger(),"rclcpp_action::exceptions::UnknownGoalHandleError");
                    }
                    RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::YAW_ALIGN Goal canceled");

            } 
            
        default:
            RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::onHalted:: No goal to cancel");
            return ;
    }
  
}

void RecoveryNode::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    odom_msg = msg;
}

void RecoveryNode::ballpose_callback(const oakd_msgs::msg::StatePose &msg)
{
    ball_pose = msg;
    if ( ball_pose.is_tracked.data)
    {
        spin_counter = 0;
        EulerAngles e;
        e = ToEulerAngles(ball_pose.goalpose.pose.orientation.w,
                          ball_pose.goalpose.pose.orientation.x,
                          ball_pose.goalpose.pose.orientation.y,
                          ball_pose.goalpose.pose.orientation.z);
        
        if (e.yaw - previous_ball_theta > 0)
        {
            ball_drift = CLOCKWISE;
        }
        else if (e.yaw - previous_ball_theta <0)
        {
            ball_drift = ANTI_CLOCKWISE;
        }
        else 
        {
            ball_drift = NO_DRIFT;
        }

        previous_ball_theta = e.yaw;
    }
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

void RecoveryNode::backUp_result_callback(const GoalHandleBackUp::WrappedResult &wrappedresult)
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
    GoalHandleBackUp::SharedPtr,
    const std::shared_ptr<const BackUp::Feedback> feedback)
{
    (void)feedback;
}

void RecoveryNode::backUp_goal_response_callback(const rclcpp_action::ClientGoalHandle<BackUp>::SharedPtr & goal_handle_)
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

void RecoveryNode::spin_result_callback(const GoalHandleSpin::WrappedResult &wrappedresult)
{
    if(wrappedresult.result)
    {
        done_flag=true;
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: Spin complete");

    }
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode:: Spin failed");

    }
}

void RecoveryNode::spin_feedback_callback(
    GoalHandleSpin::SharedPtr,
    const std::shared_ptr<const Spin::Feedback> feedback)
{
    (void)feedback;
}

void RecoveryNode::spin_goal_response_callback(const rclcpp_action::ClientGoalHandle<Spin>::SharedPtr & goal_handle_)
{
    if (!goal_handle_) 
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "RecoveryNode::Spin goal rejected ");
    } 
    else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Spining ");
        this->spin_goal_handle = goal_handle_;
    }
}

void RecoveryNode::align_yaw_goal_response_callback(const GoalHandleLineFollow::SharedPtr & goal_handle_)
{
    if (!goal_handle_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "RecoveryNode::Align Yaw goal rejected");
    } else
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "RecoveryNode::Aligning yaw");
        this->align_yaw_goal_handle = goal_handle_;
    }
}

void RecoveryNode::align_yaw_feedback_callback( 
    GoalHandleLineFollow::SharedPtr, 
    const std::shared_ptr<const LineFollow::Feedback> feedback)
{
    (void)feedback;
}

void RecoveryNode::align_yaw_result_callback(const GoalHandleLineFollow::WrappedResult & wrappedresult)
{
    if (wrappedresult.result->robot_state == wrappedresult.result->ALIGNED_YAW )
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::YAW Aligned");
    }

}


