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

    subscription_odometry = node_ptr_->create_subscription<nav_msgs::msg::Odometry>( 
        "/odometry/filtered",
        qos_profile,
        std::bind(&RecoveryNode::odometry_callback,this,std::placeholders::_1)
    );
    RCLCPP_INFO(node_ptr_->get_logger(),"RecoveryNode::Ready");
    done_flag = false;
    // RecoveryState recoveryType = HALT;
}

BT::NodeStatus RecoveryNode::onStart()
{
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


    // make nav goal pose
    auto goal_msg = NavigateToPose::Goal();
        
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = -1.80;
    goal_msg.pose.pose.orientation.z = 0.7071068;
    goal_msg.pose.pose.orientation.w = 0.7071068;

    // make backUp goal distance
    auto  backUp_dist = BackUp::Goal();

    backUp_dist.target.x = 0.5;
    backUp_dist.target.y = 0.0;
    backUp_dist.target.z = 0.0;
    backUp_dist.speed = 1.0;

    // send goal
    if ( odom_msg.pose.pose.position.y <= -1.60)
        nav_action_client_ptr_->async_send_goal(goal_msg, nav_send_goal_options);    
    else 
        backUp_action_client_ptr_->async_send_goal(backUp_dist, backUp_send_goal_options);    


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
    // auto cancel_future= action_client_ptr_->async_cancel_all_goals();
    RCLCPP_WARN(node_ptr_->get_logger(),"RecoveryNode::Halted");
}

void RecoveryNode::odometry_callback(const nav_msgs::msg::Odometry &msg)
{
    odom_msg = msg;
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
    // RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode::Navigating to origin..");
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
    // RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode::Navigating to origin..");
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
    // RCLCPP_INFO(node_ptr_->get_logger()," RecoveryNode::Navigating to origin..");
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

