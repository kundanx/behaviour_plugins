#include "ballDetection.hpp"

ballDetection::ballDetection(rclcpp::Node::SharedPtr node_ptr)
: is_detected(false), ball_inside(false), only_ball(false),node_ptr_(node_ptr)
{
    ballDetection_ = std::make_unique<ballDetection>(shared_from_this());
    subscription_1 = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "/ColorSensor_status", 10, std::bind(&ballDetection::subscriber1_callback,this,std::placeholders::_1));
    subscription_2 = node_ptr_->create_subscription<std_msgs::msg::Bool>( "/is_ball_tracked", 10, std::bind(&ballDetection::subscriber2_callback,this,std::placeholders::_1));
}


BT::NodeStatus ballDetection::isBallInside()
{
    if(ball_inside){
        RCLCPP_INFO(node_ptr_->get_logger(),"Ball Inside.");
        ball_inside = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Ball not Inside.");
    return BT::NodeStatus::FAILURE;

}
BT::NodeStatus ballDetection::isBallDetected()
{
    if(is_detected){
        RCLCPP_INFO(node_ptr_->get_logger(),"Ball Detected.");
        is_detected = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Ball not Detected.");
    return BT::NodeStatus::FAILURE;

}
BT::NodeStatus ballDetection::isOnlyBall()
{
    if(only_ball){
        RCLCPP_INFO(node_ptr_->get_logger(),"Only ball");
        only_ball = false;
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"Not only ball");
    return BT::NodeStatus::FAILURE;

}
void ballDetection::subscriber1_callback(std_msgs::msg::UInt8 msg)
{
    if(msg.data == 0xA1)
        ball_inside = true;    
    else if(msg.data == 0xA2)
        only_ball = true;
    else 
    {
        ball_inside = false;
        only_ball = false;
    }

}
void ballDetection::subscriber2_callback(std_msgs::msg::Bool msg)
{
    if(msg.data)
        is_detected = true;    
    else
        is_detected = false;
}