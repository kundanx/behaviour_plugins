#include "conditions/isBallInside.hpp"   

isBallInside::isBallInside(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "is_ball_inside", qos_profile, std::bind(&isBallInside::subscriber_callback,this,std::placeholders::_1));

    last_inside_time = get_tick_ms();
    inside = false;
    print_log = false;
    capture_inside_time = true;
    RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::Ready");

}
BT::PortsList isBallInside::providedPorts()
{
    return {BT::OutputPort<bool>("OP_IsBallInside")};
}

 BT::NodeStatus isBallInside::tick()
 {  
    if(node_called_once)
    {
        // inside = false;
        node_called_once = false;
        print_log= false;


    }
    uint32_t now = get_tick_ms();
    if ((now - last_inside_time) >= 2500 )
    {
        inside = false;
        capture_inside_time = true;
    }

    if(inside)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::yes");
        setOutput<bool>("OP_IsBallInside", inside);

        // inside = false;
        node_called_once = true;
        return BT::NodeStatus::SUCCESS;
    }
    if( !print_log)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"isBallInside::Ball not Inside.");
        print_log = true;

    }
    return BT::NodeStatus::FAILURE;
 }

void isBallInside::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(capture_inside_time)
    {
        if(msg.data == 1)
        {
            inside = true;  
            capture_inside_time = false;
            last_inside_time = get_tick_ms();  
        
        }
        else
            inside = false;
    }
}

