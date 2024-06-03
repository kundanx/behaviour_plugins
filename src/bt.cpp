

#include "bt.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("behaviour_plugins") + "/bt_xml";

/*******************************************************************************************************************
 * @class autonomy
 * @brief A class to create and handle behavior trees
 *******************************************************************************************************************/

autonomy::autonomy(const std::string &nodeName): Node(nodeName)
{   
    this->declare_parameter("location_file","none");
}

/*******************************************************************************************************************
 * @brief create behavior tree
 * @brief Update behavior tree every 50ms
********************************************************************************************************************/

void autonomy::setup()
{   
    create_behavior_tree();
    const auto timer_period = 50ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&autonomy::update_behavior_tree, this)
    );
    RCLCPP_INFO(get_logger(),"Setup_done");
}

/*******************************************************************************************************************
 * @brief Register tree nodes and Create tree 
 * @brief Register action client nodes
********************************************************************************************************************/

void autonomy::create_behavior_tree()
{   
    register_actionClient_nodes();
    register_control_nodes();
    register_decorator_nodes();
    register_custom_action_nodes();
    register_condition_nodes();

    /* Register fibbonacci_action node*/
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {   
        return std::make_unique<Fib>(name, config, shared_from_this());
    };
    factory.registerBuilder<Fib>("Fib",builder_1);

    /* Register GoToBallPose node */
    BT::NodeBuilder builder_2 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToBallPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToBallPose>("GoToBallPose",builder_2);

    /* Register GetBallPose node */ 
    BT::NodeBuilder builder_3  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GetBallPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GetBallPose>("GetBallPose",builder_3);

    /* Register isBallDetected node */ 
    BT::NodeBuilder builder_4  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<isBallDetected>(name, config, shared_from_this());
    };
    factory.registerBuilder<isBallDetected>("isBallDetected",builder_4);

    BT::NodeBuilder builder_5  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<PacketPublisher>(name, config, shared_from_this());
    };
    factory.registerBuilder<PacketPublisher>("PacketPublisher",builder_5);

     BT::NodeBuilder builder_6  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<InitiallizeActuators>(name, config,shared_from_this());
    };
    factory.registerBuilder<InitiallizeActuators>("InitiallizeActuators",builder_6);
    
     BT::NodeBuilder builder_7 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToSiloPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToSiloPose>("GoToSiloPose",builder_7);

     BT::NodeBuilder builder_8 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<LineFollower>(name, config, shared_from_this());
    };
    factory.registerBuilder<LineFollower>("LineFollower",builder_8);

    BT::NodeBuilder builder_9 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToOrigin>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToOrigin>("GoToOrigin",builder_9);

    BT::NodeBuilder builder_10 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<ResetOdom>(name, config, shared_from_this());
    };
    factory.registerBuilder<ResetOdom>("ResetOdom",builder_10);

    /* create BT */
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/BallFollower_tree.xml");

    // Connect the Groot2Publisher. This will allow Groot2 to
    // get the tree and poll status updates.
    // const unsigned port = 1667;
    // BT::Groot2Publisher publisher(tree_, port);
}

/*******************************************************************************************************************
 * @brief Tick tree nodes
********************************************************************************************************************/

void autonomy::update_behavior_tree()
{
    // tick BT when asked
    BT::NodeStatus tree_status = tree_.tickRoot();
    if(tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if(tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(),"Finished behaviour");
        timer_->cancel();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Tree Failed");
        timer_->cancel();
    }
}

/*******************************************************************************************************************
 * @brief Register custom action nodes
********************************************************************************************************************/

void autonomy::register_custom_action_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<TurnOnRoller>(name,config);
    };
    factory.registerBuilder<TurnOnRoller>("TurnOnRoller",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<TurnOnConveyer>(name,config);
    };
    factory.registerBuilder<TurnOnConveyer>("TurnOnConveyer",builder_2);

    BT::NodeBuilder builder_3  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<TurnOffRoller>(name,config);
    };
    factory.registerBuilder<TurnOffRoller>("TurnOffRoller",builder_3);

    BT::NodeBuilder builder_4  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<TurnOffConveyer>(name,config);
    };
    factory.registerBuilder<TurnOffConveyer>("TurnOffConveyer",builder_4);

    BT::NodeBuilder builder_5  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<PneumaticOn>(name,config);
    };
    factory.registerBuilder<PneumaticOn>("PneumaticOn",builder_5);

    BT::NodeBuilder builder_6  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<PneumaticOff>(name,config);
    };
    factory.registerBuilder<PneumaticOff>("PneumaticOff",builder_6);
}

/******************************************************************************************************************
 * @brief Register action client nodes
*******************************************************************************************************************/

void autonomy::register_actionClient_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<spinActionClient>(name, config, shared_from_this());
    };
    factory.registerBuilder<spinActionClient>("spinActionClient",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<waitActionClient>(name, config, shared_from_this());
    };
    factory.registerBuilder<waitActionClient>("waitActionClient",builder_2);
}


/******************************************************************************************************************
 * @brief Register control node from nav2_behavior_tree
*******************************************************************************************************************/

void autonomy::register_control_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::RecoveryNode>(name,config);
    };
    factory.registerBuilder<nav2_behavior_tree::RecoveryNode>("RecoveryNode",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::PipelineSequence>(name,config);
    };
    factory.registerBuilder<nav2_behavior_tree::PipelineSequence>("PipelineSequence",builder_2);

    BT::NodeBuilder builder_3  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::RoundRobinNode>(name,config);
    };
    factory.registerBuilder<nav2_behavior_tree::RoundRobinNode>("RoundRobin",builder_3);

    // BT::NodeBuilder builder_4  =
    //     [=](const std::string &name, const BT::NodeConfiguration &config)
    // {
    //     return std::make_unique<ParallelNode>(name,config);
    // };
    // factory.registerBuilder<ParallelNode>("ParallelNode",builder_4);

}


/******************************************************************************************************************
 * @brief Register condition nodes
*******************************************************************************************************************/

void autonomy::register_condition_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<isBallInside>(name, config, shared_from_this());
    };
    factory.registerBuilder<isBallInside>("isBallInside",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<isOnlyBall>(name, config, shared_from_this());
    };
    factory.registerBuilder<isOnlyBall>("isOnlyBall",builder_2);
}


/******************************************************************************************************************
 * @brief Register decorator nodes
*******************************************************************************************************************/

void autonomy::register_decorator_nodes()
{   
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<returnSuccess>(name,config);
    };
    factory.registerBuilder<returnSuccess>("returnSuccess",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<returnFailure>(name,config);
    };
    factory.registerBuilder<returnFailure>("returnFailure",builder_2);
}


/******************************************************************************************************************
 * @brief Main function 
*******************************************************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<autonomy>("Autonomy_node");
    node->setup();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}