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
    // create BT
    BT::BehaviorTreeFactory factory;
    // RCLCPP_INFO(get_logger(),"kuns");
    auto nh = std::make_shared<rclcpp::Node>("kuns");
    
    RosNodeParams params;
    params.nh = nh;
    // RCLCPP_INFO(get_logger(),"okss");
    params.default_port_value = "ball_pose_topic";  // Specify topic here

    factory.registerNodeType<RecieveGoalPose>("RecieveGoalPose", params);

    BT::NodeBuilder builder_1 =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToPose>("GoToPose",builder_1);

    BT::NodeBuilder builder_2  =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<Fib>(name, config, shared_from_this());
    };
    factory.registerBuilder<Fib>("Fib",builder_2);

    

    // BT::NodeBuilder builder_3  =
    //     [=](const std::string &name, const BT::NodeConfiguration &config)
    // {
    //     return std::make_unique<RecieveGoalPose>(name, config, shared_from_this());
    // };
    // factory.registerBuilder<RecieveGoalPose>("RecieveGoalPose",builder_3);
    
    // RCLCPP_INFO(get_logger(),"siwa");
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/bt_tree.xml");

   // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
    const unsigned port = 1667;
    BT::Groot2Publisher publisher(tree_, port);
}

void autonomy::update_behavior_tree()
{
    // tick BT when asked
    BT::NodeStatus tree_status = tree_.tickOnce();
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

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<autonomy>("Autonomy_node");
    node->setup();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}