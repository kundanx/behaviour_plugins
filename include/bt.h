#ifndef BT_H
#define BT_H

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "isBallDetected.hpp"
#include "GetBallPose.hpp"
#include "GoToBallPose.h"
#include "fib_behaviour.h"
#include "spinActionClient.hpp"
#include "waitActionClient.hpp"   

#include "actions/TurnOnRoller.hpp"
#include "actions/TurnOffRoller.hpp"

#include "actions/TurnOnConveyer.hpp"
#include "actions/TurnOffConveyer.hpp"

#include "actions/PneumaticOn.hpp"
#include "actions/PneumaticOff.hpp"

#include "control/pipeline_sequence.hpp"
#include "control/recovery_node.hpp"
#include "control/round_robin_node.hpp"

#include "nav2_behavior_tree/plugins/action/spin_action.hpp"


class autonomy : public rclcpp::Node
{
    public:
    explicit autonomy(const std::string &node_name);
    void setup(); // to initiallize everything
    void create_behavior_tree();
    void update_behavior_tree();

    void register_action_nodes();
    void register_control_nodes();
    void register_actionClient_nodes();

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory;
};

#endif
