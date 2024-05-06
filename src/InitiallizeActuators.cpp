#include <chrono>
#include "InitiallizeActuators.hpp"  

using namespace std::chrono_literals;

InitiallizeActuators::InitiallizeActuators(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    const auto timer_period = 30ms;
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>( "act_vel", 10);
    subscription_ = node_ptr_->create_subscription<std_msgs::msg::UInt8>( 
            "ball_roller_status",
            10,
            std::bind(&InitiallizeActuators::subscriber_callback,this,std::placeholders::_1));
    timer_ = node_ptr_->create_wall_timer(
        timer_period,
        std::bind(&InitiallizeActuators::timer_callback, this)
    );
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators node Ready..");
}

BT::PortsList InitiallizeActuators::providedPorts()
{
    return {
        BT::InputPort<bool>("Ip_RollerStatus"),
        BT::InputPort<int>("Ip_RollerSpeed"),   
        BT::InputPort<bool>("Ip_ConveyerStatus"),
        BT::InputPort<int>("Ip_ConveyerSpeed"), 
        BT::InputPort<bool>("Ip_PneumaticStatus"),

        BT::OutputPort<bool>("Op_IsBallInside"),
        BT::OutputPort<bool>("Op_IsOnlyBall")
    };
}

 BT::NodeStatus InitiallizeActuators::tick()
 {  
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(3);
    msg.data[0]= 0;
    msg.data[1]= 0;
    msg.data[2]=0x00;
    publisher_->publish(msg);
    setOutput<bool>("Op_IsBallInside", false);
    setOutput<bool>("Op_IsOnlyBall", false);
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Actuators in reset state..");
    return BT::NodeStatus::SUCCESS;
 }

 void InitiallizeActuators::subscriber_callback(std_msgs::msg::UInt8 msg)
 {
    // RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators subs callback..");
    switch (msg.data)
    {   
        // Red ball
        case 1: 
            // RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Red Ball detected");
            setOutput<bool>("Op_IsBallInside", true);
            break;

        // Blue ball
        case 2:
            // RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Blue Ball detected");
            setOutput<bool>("Op_IsBallInside", false);
            break;

        // Purple ball
        case 3:
            // RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"purple Ball detected");
            setOutput<bool>("Op_IsBallInside", false);
            break;

        // No ball
        default:
            // RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"No ball detected");
            setOutput<bool>("Op_IsBallInside", false);
    }
 }

 void InitiallizeActuators::timer_callback()
 {
    auto ConveyerStatus_ = getInput<bool>("Ip_ConveyerStatus");
    auto ConveyerSpeed_ = getInput<int>("Ip_ConveyerSpeed");
    auto RollerStatus_ = getInput<bool>("Ip_RollerStatus");
    auto RollerSpeed_ = getInput<int>("Ip_RollerSpeed");
    auto PneumaticStatus_ = getInput<bool>("Ip_PneumaticStatus");

    if ( !(RollerSpeed_ && ConveyerStatus_ && ConveyerSpeed_ && RollerStatus_ && PneumaticStatus_) )
    {
        throw BT::RuntimeError("[InitiallizeActuators] error reading port");
    }
    auto ConveyerStatus = ConveyerStatus_.value();
    auto ConveyerSpeed = ConveyerSpeed_.value();
    auto RollerStatus = RollerStatus_.value();
    auto RollerSpeed = RollerSpeed_.value();
    auto PneumaticStatus = PneumaticStatus_.value();
    
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(3);
    msg.data[0]= RollerSpeed;
    msg.data[1]= ConveyerSpeed;
    msg.data[2]=0x00;

    if (RollerStatus)
        msg.data[2] = msg.data[2] | 0b00000010 ;
    
    if (ConveyerStatus)
        msg.data[2] = msg.data[2] | 0b00000001;
    
    if (PneumaticStatus)
        msg.data[2] = msg.data[2] | 0b00000100;

    publisher_->publish(msg);

 }

















