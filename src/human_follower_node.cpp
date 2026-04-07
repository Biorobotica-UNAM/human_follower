#include <iostream>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class HumanFollower : public rclcpp::Node {
public:
    HumanFollower() : Node("human_follower"), enable_(false) {
        // Parámetros de control (Mantengo tus valores originales que ya funcionan)
        control_alpha_  = this->declare_parameter("control_alpha", 0.6548);
        control_beta_   = this->declare_parameter("control_beta", 0.3);
        max_linear_     = this->declare_parameter("max_linear", 0.3);
        max_angular_    = this->declare_parameter("max_angular", 0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.9);
        move_backwards_ = this->declare_parameter("move_backwards", false);
        use_potfields_  = this->declare_parameter("enable_potfields", false);
        cmd_vel_topic_   = this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
        legs_pose_topic_ = this->declare_parameter("legs_pose_topic", std::string("/hri/leg_finder/leg_pose"));
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.65);
        
        // Publisher de velocidades
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/hardware/base/cmd_vel", 10);

        // TF2 setup
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Pub/Sub
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 
            rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal));
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { enable_ = msg->data; });

        // Timer de control (30ms = ~33Hz)
        timer_ = this->create_wall_timer(30ms, std::bind(&HumanFollower::control_loop, this));
        
        last_time_ = this->get_clock()->now();
    }

private:
    // --- Variables de Estado ---
    bool enable_;
    geometry_msgs::msg::PointStamped last_target_;
    bool has_target_ = false;
    
    // Perfil de velocidad
    double last_linear_vel_ = 0.0;
    double last_angular_vel_ = 0.0;
    const double MAX_ACCEL = 0.5;       // m/s^2
    const double MAX_ALPHA_ACCEL = 1.0; // rad/s^2
    
    float control_alpha_, control_beta_, max_linear_, max_angular_, dist_to_human_;
    rclcpp::Time last_time_;

    // --- Callbacks ---
    void callback_target(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_target_ = *msg;
        has_target_ = true;
    }

    void control_loop() {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (!enable_ || !has_target_ || (now - last_target_.header.stamp).seconds() > 0.5) {
            stop_robot();
            return;
        }

        // 1. Calcular velocidades deseadas (Raw)
        geometry_msgs::msg::Twist raw_vel = calculate_raw_speeds(last_target_.point.x, last_target_.point.y);

        // 2. Aplicar Rampas de Aceleración para suavidad
        geometry_msgs::msg::Twist smooth_vel;
        
        double max_dv = MAX_ACCEL * dt;
        double dv = raw_vel.linear.x - last_linear_vel_;
        last_linear_vel_ += std::clamp(dv, -max_dv, max_dv);

        double max_dw = MAX_ALPHA_ACCEL * dt;
        double dw = raw_vel.angular.z - last_angular_vel_;
        last_angular_vel_ += std::clamp(dw, -max_dw, max_dw);

        smooth_vel.linear.x = last_linear_vel_;
        smooth_vel.angular.z = last_angular_vel_;

        pub_cmd_vel_->publish(smooth_vel);
    }

    geometry_msgs::msg::Twist calculate_raw_speeds(float gx, float gy) {
        geometry_msgs::msg::Twist vel;
        float dist_total = sqrt(gx*gx + gy*gy);
        float angle_error = atan2(gy, gx);
        float dist_error = dist_total - dist_to_human_;

        // Tu lógica de control original mejorada
        if (std::abs(dist_error) > 0.05 || std::abs(angle_error) > 0.1) {
            // Velocidad lineal con decaimiento según el error de ángulo (exponencial)
            float target_lin = std::clamp(dist_error, -max_linear_, max_linear_);
            vel.linear.x = target_lin * exp(-(angle_error * angle_error) / control_alpha_);
            
            // Velocidad angular usando la sigmoide que tenías
            vel.angular.z = max_angular_ * (2.0f / (1.0f + exp(-angle_error / control_beta_)) - 1.0f);
        }
        return vel;
    }

    void stop_robot() {
        last_linear_vel_ = 0.0;
        last_angular_vel_ = 0.0;
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        has_target_ = false;
    }

    // Publishers/Subscribers...
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Parameters
    bool move_backwards_;
    bool use_potfields_;
    std::string cmd_vel_topic_;
    std::string legs_pose_topic_;

    // TF2 Objects
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
