#include <node.hpp>

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions options;

    std::shared_ptr<add_two_values::AddTwoValues> add_two_values_node = std::make_shared<add_two_values::AddTwoValues>(options);

    exec.add_node(add_two_values_node->get_node_base_interface());

    exec.spin();

    rclcpp::shutdown();

    return 0;
}
