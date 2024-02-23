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

    tree_ = factory.createTreeFromFile(bt_xml_dir + "/bt_tree.xml");
    BT::Groot2Publisher publisher(tree_);
// publiser.clearTree();
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
        timer_->cancel();
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