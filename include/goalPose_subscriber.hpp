#ifndef GOALPOSE_SUBSCRIBER_HPP
#define GOALPOSE_SUBSCRIBER_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace BT;

using PosMsg = geometry_msgs::msg::PoseStamped;

class RecieveGoalPose: public  RosTopicSubNode<PosMsg>
{
    public:
    RecieveGoalPose(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params);

    std::shared_ptr<PosMsg>  pose_;

    static BT::PortsList providedPorts();

    BT::NodeStatus onTick(const std::shared_ptr<PosMsg>& new_goal) override;

};

#endif