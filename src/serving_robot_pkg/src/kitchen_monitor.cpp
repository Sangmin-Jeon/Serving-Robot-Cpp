#include <rclcpp/rclcpp.hpp>
#include <string>

class KitchenMonitorNode : public rclcpp::Node {
public:
    KitchenMonitorNode(): Node("kitchen_monitor_node") {
        RCLCPP_INFO(this->get_logger(), "Kitchen Monitor Node has started!");
        
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KitchenMonitorNode>());
    rclcpp::shutdown();

    return 0;
}