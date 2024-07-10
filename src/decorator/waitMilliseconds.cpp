#include "decorator/waitMilliseconds.hpp"   

/*****************************************************************************************************************
 * @brief Tree Node to bring robot back to origin to intake new ball after ball is stored in silo
******************************************************************************************************************/
using namespace std::chrono_literals;

waitMillisecond::waitMillisecond(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    RCLCPP_INFO(node_ptr_->get_logger(),"waitMillisecond::Ready");
}


 BT::PortsList waitMillisecond::providedPorts()
 {
    return{BT::InputPort<uint64_t>("Ip_wait_time")};
 }

BT::NodeStatus waitMillisecond::onStart()
{
    auto wait_time_ = getInput<uint64_t>("Ip_wait_time");
    if(!wait_time_ )
    {
        throw BT::RuntimeError("[waitMillisecond] error reading Ip_wait_time");
       
    }
    wait_time = wait_time_.value();

    // Get the current time from the system clock
    auto now = std::chrono::system_clock::now();
    // Convert the current time to time since epoch
    auto duration = now.time_since_epoch();
    // Convert duration to milliseconds
    start_time = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // done_flag = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus waitMillisecond::onRunning()
{   

    auto now_ = std::chrono::system_clock::now();

    // Convert the current time to time since epoch
    auto duration = now_.time_since_epoch();
    // Convert duration to milliseconds
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    
    if((now - start_time) >= wait_time)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"Wait completed %lims", wait_time);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void waitMillisecond::onHalted() 
{
    RCLCPP_WARN(node_ptr_->get_logger(),"waitMillisecond::Navigation aborted");
}

namespace BT
{
    template <> inline uint64_t convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, '\n');
        if (parts.size() != 1)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            uint64_t output;
            output  = convertFromString<float>(parts[0]);
            return output;
        }
    }
} // end names

