#include "CvConfig.hpp"   

/*****************************************************************************************************************
 * @brief Tree Node to bring robot back to origin to intake new ball after ball is stored in silo
******************************************************************************************************************/


using namespace std::chrono_literals;

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do 
    {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {break;}
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    }
    while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}


Cv_Config::Cv_Config(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::SyncActionNode(name,config), node_ptr_(node_ptr)
{
    ball_client_ptr_ =node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/oak/yolo/yolov8_node/change_state");
    silo_client_ptr_ =node_ptr_->create_client<lifecycle_msgs::srv::ChangeState>("/silo/yolo/yolov8_node/change_state");

    auto deactive_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        deactive_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    
    auto result_1 = ball_client_ptr_->async_send_request(deactive_request).future.share();
    auto result_2 = silo_client_ptr_->async_send_request(deactive_request).future.share();

    RCLCPP_INFO(node_ptr_->get_logger(),"Cv_Config::Ready");

}

BT::PortsList Cv_Config::providedPorts()
{
    return {BT::InputPort<int>("In_vision_type")};
}

BT::NodeStatus Cv_Config::tick()
{
    auto vision_type_ = getInput<int>("In_vision_type");
    if ( ! vision_type_)
    {
        throw BT::RuntimeError("error reading port [In_vision_type]:", vision_type_.error());
    }
    vision_type = vision_type_.value();

    auto active_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    active_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    
    auto deactive_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    deactive_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

    // lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE

    if (!ball_client_ptr_->wait_for_service(50ms)) {
        RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "ball Service service not available, Returning Failure...");
        return BT::NodeStatus::FAILURE;
    }
    if (!silo_client_ptr_->wait_for_service(50ms)) {
        RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "silo Service service not available, Returning Failure...");
        return BT::NodeStatus::FAILURE;
    }
    
    if(vision_type == VisionType::EH_BALLZ)
    {
        auto result_1 = ball_client_ptr_->async_send_request(active_request).future.share();
        auto result_2 = silo_client_ptr_->async_send_request(deactive_request).future.share();

        // RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "blocking here1");
        // if (wait_for_result(result_1, 500ms )  == std::future_status::ready)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Ball Service success");
            
        //     if (wait_for_result( result_2, 500ms )  == std::future_status::ready)
        //     {
        //         RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Silo Service success");
        //         return BT::NodeStatus::SUCCESS;
    
        //     }
        //     else
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger("Cv_Config"), "Silo Service failed");   
        //     }
        // }
        // else
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("Cv_Config"), "Ball Service failed");   
        // }
        RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Silo Service success");

        return BT::NodeStatus::SUCCESS;

        
     
    }
    else if (vision_type == VisionType::SILO_DETECTION)
    {
        auto result_1 = ball_client_ptr_->async_send_request(deactive_request);
        auto result_2 = silo_client_ptr_->async_send_request(active_request);


        // RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "blocking here2");
        // if (wait_for_result( result_1, 500ms )  == std::future_status::ready)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Ball Service success");
        //     if (rclcpp::spin_until_future_complete(node_ptr_, result_2) ==
        //         rclcpp::FutureReturnCode::SUCCESS)
        //     {
        //         RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Silo Service success");
        //         return BT::NodeStatus::SUCCESS;
        //     }
        //       else
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger("Cv_Config"), "Silo Service failed");   
        //     }
        // }
        // else
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("Cv_Config"), "Ball Service failed");   
        // }
        RCLCPP_INFO(rclcpp::get_logger("Cv_Config"), "Ball Service success");

        return BT::NodeStatus::SUCCESS;
       
    }
    return BT::NodeStatus::FAILURE;
}





