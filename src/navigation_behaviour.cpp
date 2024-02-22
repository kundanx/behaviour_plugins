#include "navigation_behaviour.h"

GoToPose::GoToPose(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node_ptr) : BT::StatefulActionNode(name,config), node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
    done_flag = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
    BT::Expected<std::string> loc = getInput<std::string>("loc");
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

    YAML::Node locations = YAML::LoadFile(location_file);
    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    // Setup action client goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    // make pose
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];

    tf2::Quaternion q;
    q.setRPY(0,0,pose[2]);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // send pose
    done_flag = false;
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr_->get_logger(),"sent goal to nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToPose::onRunning()
{   
    if(done_flag)
    {
        RCLCPP_INFO(node_ptr_->get_logger(),"[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if(result.result)
    {
        done_flag=true;
    }
}