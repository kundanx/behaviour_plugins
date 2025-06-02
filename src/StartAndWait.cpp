#include "StartAndWait.hpp"   

StartAndWait::StartAndWait(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( "go_or_wait", qos_profile, std::bind(&StartAndWait::subscriber_callback,this,std::placeholders::_1));
    RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::Ready..");
    wait = true;

}

BT::PortsList StartAndWait::providedPorts()
{
    return {BT::OutputPort<int>("Op_Start_wait"),

            BT::OutputPort<bool>("Op_RollerStatus"),
            BT::OutputPort<int>("Op_RollerSpeed"),   
            BT::OutputPort<bool>("Op_ConveyerStatus"),
            BT::OutputPort<int>("Op_ConveyerSpeed"), 
            BT::OutputPort<bool>("Op_PneumaticStatus"),
            BT::OutputPort<bool>("OP_IsBallInside"),
            BT::OutputPort<bool>("OP_IsOnlyBall")
        };
}

 BT::NodeStatus StartAndWait::tick()
 {  
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);
    setOutput<bool>("Op_RollerStatus", false);
    setOutput<int>("Op_RollerSpeed", 1);
    setOutput<bool>("Op_PneumaticStatus", false);
    if(!wait && start)
    {
        // RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::Start");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),"StartAndWait::WAIT");
    return BT::NodeStatus::FAILURE;
 }

void StartAndWait::subscriber_callback(std_msgs::msg::UInt8 msg)
{   
    if(msg.data == 0x0f && wait )
    {
        start = true;    
        wait = false;
        setOutput<int>("Op_Start_wait", 1);

    }
    else if ( msg.data == 0xf0)
    {
        start = false;    
        wait = true;
        setOutput<int>("Op_Start_wait", -1);  
    }
}
