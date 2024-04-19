#include "bt.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("behaviour_plugins") + "/bt_xml";

autonomy::autonomy(const std::string &nodeName): Node(nodeName)
{   
    this->declare_parameter("location_file","none");
    RCLCPP_INFO(this->get_logger(),"Init_done");
    RCLCPP_INFO(get_logger(),"Constructor");
}

void autonomy::setup()
{   
    RCLCPP_INFO(get_logger(),"Inside setup");
    // initial BT setup
    create_behavior_tree();
    RCLCPP_INFO(get_logger(),"BT created");

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&autonomy::update_behavior_tree, this)
    );
    RCLCPP_INFO(get_logger(),"Setup_done");

}

void autonomy::create_behavior_tree()
{   
    // register_action_nodes();
    register_control_nodes();

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


    /* Register spinActionClient node */ 
    BT::NodeBuilder builder_5  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<spinActionClient>(name, config, shared_from_this());
    };
    factory.registerBuilder<spinActionClient>("spinActionClient",builder_5);



    /* create BT */
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/BallFollower_tree.xml");
    RCLCPP_INFO(get_logger(),"kuns4");


    // Connect the Groot2Publisher. This will allow Groot2 to
    // get the tree and poll status updates.
    // const unsigned port = 1667;
    // BT::Groot2Publisher publisher(tree_, port);
}

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
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Navigation failed");
        // timer_->cancel();
    }

}


void autonomy::register_action_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::SpinAction>(name,"Spin",config);
    };
    factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin",builder_1);
}

void autonomy::register_control_nodes()
{
    BT::NodeBuilder builder_1  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::RecoveryNode>(name,config);
    };
    factory.registerBuilder<nav2_behavior_tree::RecoveryNode>("RecoveryNode",builder_1);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<autonomy>("Autonomy_node");
    node->setup();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}