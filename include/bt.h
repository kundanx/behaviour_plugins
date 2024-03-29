#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "navigation_behaviour.h"
#include "fib_behaviour.h"

class autonomy : public rclcpp::Node
{
    public:
    explicit autonomy(const std::string &node_name);
    void setup(); // to initiallize everything
    void create_behavior_tree();
    void update_behavior_tree();

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
};
