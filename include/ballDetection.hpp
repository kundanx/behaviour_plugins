#include <string>
#include <memory>


#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

class ballDetection
{   
    private:
    bool is_detected;
    bool ball_inside;
    bool only_ball;

    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> subscription_1;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> subscription_2;


    public:
    ballDetection(rclcpp::Node::SharedPtr node_ptr);

    BT::NodeStatus isBallInside();
    BT::NodeStatus isBallDetected();
    BT::NodeStatus isOnlyBall();

    void subscriber1_callback(std_msgs::msg::UInt8 msg);
    void subscriber2_callback(std_msgs::msg::Bool msg);
};