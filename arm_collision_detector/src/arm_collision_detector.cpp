#include "arm_collision_detector/arm_collision_detector.hpp"

namespace collision_detector {

CollisionDetector::CollisionDetector() : 
    Node("collision_detector_node") 
{
    
    declare_parameter("collision_gain", 10.0);
    gain_ = get_parameter("collision_gain").as_double();

    declare_parameter("collision_residual_threshold", 1.0);
    residual_threshold_ = get_parameter("collision_residual_threshold").as_double();

    last_state_stamp_ = now();

    // Inicialização do Subscriber: Tópico "joint_states"
    joint_states_sub_ = this->create_subscription<JointStatesMsg>(
        "joint_states", 10, std::bind(
            &CollisionDetector::joint_states_callback, this, std::placeholders::_1
        )
    );

    // Inicialização do Publisher: Tópico "joint_collision"
    collision_state_pub_ = this->create_publisher<JointCollisionMsg>("joint_collision", 10);

    RCLCPP_INFO(this->get_logger(), "Collision Detector Node iniciado.");
}

void CollisionDetector::joint_states_callback(const JointStatesMsg::SharedPtr msg) {
    // 1. Extrair os esforços (torque) da mensagem
    if (msg->effort.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Recebido JointState sem dados de esforço!");
        return;
    }

    if (residual_.size() != msg->effort.size()) {
        residual_.resize(msg->effort.size());
    }
    

    // 2. Calcular o torque residual
    residual_ = calculate_residual_torque(residual_, msg->effort, msg->header.stamp);

    // 3. Identificar se houve colisão
    const std::vector<bool> collision = identify_collision(residual_);

    auto collision_msg = JointCollisionMsg();
    collision_msg.header.stamp = msg->header.stamp;
    collision_msg.header.frame_id = msg->header.frame_id;
    collision_msg.joint_names = msg->name;
    collision_msg.residual = residual_;
    collision_msg.collision = collision;
    collision_state_pub_->publish(collision_msg);
}

std::vector<double> CollisionDetector::calculate_residual_torque(
    const std::vector<double> &residual, 
    const std::vector<double> &efforts, 
    const rclcpp::Time &stamp
) {
    
    if (residual.size() != efforts.size()) {
        return std::vector<double>();
    }

    std::vector<double> new_residual = residual;
    double dt = (stamp - last_state_stamp_).seconds();
    last_state_stamp_ = stamp;

    RCLCPP_DEBUG_STREAM(get_logger(), "DT = " << dt << " s");
    RCLCPP_DEBUG_STREAM(get_logger(), "Gain = " << gain_);
    
    // r' = - Kr + KTau
    for (size_t i = 0; i < efforts.size(); i++) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Joint " << i << ":");
        RCLCPP_DEBUG_STREAM(get_logger(), "  Residual = " << residual[i]);
        RCLCPP_DEBUG_STREAM(get_logger(), "  Effort = " << efforts[i]);

        new_residual[i] = residual[i] + (- gain_ * residual[i] + gain_ * efforts[i]) * dt;
        RCLCPP_DEBUG_STREAM(get_logger(), "  New residual = " << new_residual[i] << "\n");
    }

    return new_residual;
}

std::vector<bool> CollisionDetector::identify_collision(const std::vector<double> &residual) {
    std::vector<bool> in_collision;
    in_collision.reserve(residual.size());
    
    for (size_t i = 0; i < residual.size(); i++) {
        const bool collision = abs(residual[i]) > residual_threshold_;
        in_collision.push_back(collision);
    }
    
    return in_collision;
}

} // namespace collision_detector
