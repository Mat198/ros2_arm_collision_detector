#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace collision_detector {

using JointStatesMsg = sensor_msgs::msg::JointState;

class CollisionDetector : public rclcpp::Node {

public:

    CollisionDetector();

    void joint_states_callback(const JointStatesMsg::SharedPtr msg);

    void calculate_residual_torque(const std::vector<double> &efforts);

    void identify_collision();

private:
    
    rclcpp::Subscription<JointStatesMsg>::SharedPtr m_jointStatesSub;
    rclcpp::Publisher</* msg_type */>::SharedPtr m_collisionStatePub;

};
}  // namespace collision_detector
