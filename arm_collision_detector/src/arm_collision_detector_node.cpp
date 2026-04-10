#include "rclcpp/rclcpp.hpp"
#include "arm_collision_detector/arm_collision_detector.hpp"

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<collision_detector::CollisionDetector>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
