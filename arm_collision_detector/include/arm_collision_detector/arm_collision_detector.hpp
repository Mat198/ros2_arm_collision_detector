#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_collision_detector_msgs/msg/joint_collision.hpp"

namespace collision_detector {

using JointStatesMsg = sensor_msgs::msg::JointState;
using JointCollisionMsg = arm_collision_detector_msgs::msg::JointCollision;

class CollisionDetector : public rclcpp::Node {

public:

    CollisionDetector();

    void joint_states_callback(const JointStatesMsg::SharedPtr msg);

    std::vector<double> calculate_residual_torque(
        const std::vector<double> &residual, 
        const std::vector<double> &efforts,
        const rclcpp::Time &stamp
    );

    std::vector<bool> identify_collision(const std::vector<double> &residual);

private:
    
    rclcpp::Subscription<JointStatesMsg>::SharedPtr joint_states_sub_;
    rclcpp::Publisher<JointCollisionMsg>::SharedPtr collision_state_pub_;

    std::vector<double> residual_;
    double gain_;
    double residual_threshold_;
    rclcpp::Time last_state_stamp_;
};
}  // namespace collision_detector
