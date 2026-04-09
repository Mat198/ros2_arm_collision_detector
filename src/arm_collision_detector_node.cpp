#include "rclcpp/rclcpp.hpp"
#include "arm_colision_detector/arm_colision_detector.hpp"

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionDetecotor::CollisionDetecotor>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
