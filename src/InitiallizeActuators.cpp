#include "InitiallizeActuators.hpp"   

/*****************************************************************************************************************
 * @brief Initiallize actuators to resest state
 * @brief Start publisher to publish actutator commands
 * @brief Publisher : act_vel_cmd
******************************************************************************************************************/

InitiallizeActuators::InitiallizeActuators(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"InitiallizeActuators node Ready..");
}

BT::PortsList InitiallizeActuators::providedPorts()
{
    return {
        BT::OutputPort<bool>("Op_RollerStatus"),
        BT::OutputPort<int>("Op_RollerSpeed"),   
        BT::OutputPort<bool>("Op_ConveyerStatus"),
        BT::OutputPort<int>("Op_ConveyerSpeed"), 
        BT::OutputPort<bool>("Op_PneumaticStatus"),
        BT::OutputPort<bool>("OP_IsBallInside"),
        BT::OutputPort<bool>("OP_IsOnlyBall")
    };
}

 BT::NodeStatus InitiallizeActuators::tick()
 {  
    setOutput<bool>("Op_ConveyerStatus", false);
    setOutput<int>("Op_ConveyerSpeed", 1);
    setOutput<bool>("Op_RollerStatus", false);
    setOutput<int>("Op_RollerSpeed", 1);
    setOutput<bool>("Op_PneumaticStatus", false);
    RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"),"Actuators in reset state..");

    int ball_detect_status = system("exit $(ros2 lifecycle  get /oak/yolo/yolov8_node | grep -oP '(?<=\[).*?(?=\])')");
    int silo_detect_status = system("exit $(ros2 lifecycle  get /silo/yolo/yolov8_node | grep -oP '(?<=\[).*?(?=\])')");

    if ( ball_detect_status != 3)           
    {
        int ball_flag = system("ros2 lifecycle set /oak/yolo/yolov8_node activate ");
        RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"), "ball: %i", ball_flag);
    }


    if ( silo_detect_status == 3)
    {
        int silo_flag = system("ros2 lifecycle set /silo/yolo/yolov8_node deactivate ");
        RCLCPP_INFO(rclcpp::get_logger("InitiallizeActuators"), "silo: %i", silo_flag);
    }

    
    return BT::NodeStatus::SUCCESS;
 }


















