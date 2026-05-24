// ============================================================================
//  human_follower_nodeV2.cpp  —  RoboCup-grade version
//  Fixes vs original:
//    [BUG-1] no_legs_++ y no_body_++ se ejecutaban SIEMPRE al final de
//            control_loop(), incluso cuando se acababan de recibir datos válidos.
//            Movidos exclusivamente al bloque else (sin datos).
//    [BUG-2] La tracking_gate_ fija (0.45 m) es demasiado restrictiva cuando
//            el robot se mueve. Ahora es adaptiva: se amplía en función de la
//            velocidad lineal del robot y de la incertidumbre del Kalman (traza P).
//  Mejoras:
//    [OPT-1] Recovery mode: cuando se pierde al humano, el robot gira lentamente
//            buscándolo en lugar de pararse abruptamente. Se activa tras
//            `recovery_start_frames_` ciclos sin detección.
//    [OPT-2] Dead-band dinámica en distancia: la zona muerta crece si la
//            incertidumbre del Kalman es alta (P traza grande), evitando
//            oscilaciones cuando la estimación es ruidosa.
//    [OPT-3] Publicación de estado de la máquina de estados (FOLLOWING /
//            RECOVERY / STOPPED) para diagnóstico externo.
//    [OPT-4] Publicación de la covarianza del Kalman como diagnóstico.
//    [OPT-5] Normalización de pesos de fusión para garantizar que sumen 1.0
//            independientemente de los valores que configure el usuario.
// ============================================================================

#include <iostream>
#include <cmath>
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;

// Máquina de estados
enum class FollowerState { STOPPED, FOLLOWING, RECOVERY };

static std::string state_to_str(FollowerState s) {
    switch (s) {
        case FollowerState::STOPPED:   return "STOPPED";
        case FollowerState::FOLLOWING: return "FOLLOWING";
        case FollowerState::RECOVERY:  return "RECOVERY";
    }
    return "UNKNOWN";
}

class HumanFollower : public rclcpp::Node
{
public:
    HumanFollower()
    : Node("human_follower"), enable_(false), state_(FollowerState::STOPPED)
    {
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER ---");

        // Parametros
        control_alpha_  = this->declare_parameter("control_alpha",       0.6548);
        control_beta_   = this->declare_parameter("control_beta",        0.3);
        max_linear_     = this->declare_parameter("max_linear",          0.3);
        max_angular_    = this->declare_parameter("max_angular",         0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human",  0.65);
        move_backwards_ = this->declare_parameter("move_backwards",      false);
        cmd_vel_topic_  = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

        leg_weight_     = this->declare_parameter("fusion.leg_weight",   0.55);
        body_weight_    = this->declare_parameter("fusion.body_weight",  0.45);
        tracking_gate_  = this->declare_parameter("fusion.tracking_gate", 0.45);

        // Param de recovery ───────────────────────────────────
        recovery_start_frames_  = this->declare_parameter("recovery.start_frames",  8);
        recovery_stop_frames_   = this->declare_parameter("recovery.stop_frames",  30);
        recovery_angular_vel_   = this->declare_parameter("recovery.angular_vel",  0.25);

        // Gate adaptiva
        gate_vel_factor_   = this->declare_parameter("fusion.gate_vel_factor",   0.3);
        gate_trace_factor_ = this->declare_parameter("fusion.gate_trace_factor", 0.1);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Suscriptore
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            std::bind(&HumanFollower::callback_enable, this, std::placeholders::_1));
        sub_legs_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/hri/leg_finder/legs_pose", 1,
            std::bind(&HumanFollower::callback_legs, this, std::placeholders::_1));
        sub_body_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/hri/human_tracking/target_point", 1,
            std::bind(&HumanFollower::callback_body, this, std::placeholders::_1));

        // Publicadores
        pub_cmd_vel_      = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        pub_debug_point_  = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/hri/human_follower/debug_fused_point", 10);
        //revisas el estado del seguidor    
        pub_state_        = this->create_publisher<std_msgs::msg::String>(
            "/hri/human_follower/state", 10);
        // incertidumbre Kalman
        pub_kalman_trace_ = this->create_publisher<std_msgs::msg::Float32>(
            "/hri/human_follower/kalman_uncertainty", 10);

        // Kalman de 4-estados x, vx, y, vy
        x_ = Eigen::Vector4d::Zero();
        P_ = Eigen::Matrix4d::Identity() * 0.1;
        F_ = Eigen::Matrix4d::Identity();
        H_ = Eigen::MatrixXd::Zero(2, 4);
        H_(0, 0) = 1.0;
        H_(1, 2) = 1.0;
        Q_ = Eigen::Matrix4d::Identity() * 0.02;
        R_ = Eigen::Matrix2d::Identity() * 0.05;

        last_time_         = this->get_clock()->now();
        last_linear_vel_   = 0.0;
        last_angular_vel_  = 0.0;
        no_legs_           = 0;
        no_body_           = 0;
        recovery_dir_      = 1.0;

        timer_ = this->create_wall_timer(50ms, std::bind(&HumanFollower::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
            "HumanFollower ready. Gate base=%.3f  vel_factor=%.2f  trace_factor=%.2f",
            tracking_gate_, gate_vel_factor_, gate_trace_factor_);
    }

private:
    // Param
    double control_alpha_, control_beta_;
    double max_linear_, max_angular_;
    double dist_to_human_;
    bool   move_backwards_;
    std::string cmd_vel_topic_;

    double leg_weight_, body_weight_, tracking_gate_;

    int    recovery_start_frames_, recovery_stop_frames_;
    double recovery_angular_vel_;
    double gate_vel_factor_, gate_trace_factor_;

    const double MAX_ACCEL       = 0.6;
    const double MAX_ALPHA_ACCEL = 1.2;

    // Estado de variables
    bool           enable_;
    // When disabled, publish one stop command and then stay silent so
    // navigation, align and point can own /hardware/base/cmd_vel.
    bool           disabled_stop_published_{false};
    FollowerState  state_;
    int            no_legs_, no_body_;
    double         recovery_dir_;
    rclcpp::Time   last_time_;
    double         last_linear_vel_, last_angular_vel_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           sub_enable_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_body_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_point_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_state_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_kalman_trace_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

    // Kalman de 4 estados: x, vx, y, vy
    Eigen::Vector4d  x_;
    Eigen::Matrix4d  P_;
    Eigen::Matrix4d  F_;
    Eigen::MatrixXd  H_;
    Eigen::Matrix4d  Q_;
    Eigen::Matrix2d  R_;

    geometry_msgs::msg::PointStamped current_legs_;
    geometry_msgs::msg::PointStamped current_body_;

    // Callbacks de recepción 
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
        if (enable_) {
            disabled_stop_published_ = false;
        } else {
            transition_to(FollowerState::STOPPED);
            last_linear_vel_  = 0.0;
            last_angular_vel_ = 0.0;
            publish_disabled_stop_once();
        }
    }
    void callback_legs(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        current_legs_ = *msg;
        no_legs_ = 0;
    }
    void callback_body(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        current_body_ = *msg;
        no_body_ = 0;
    }

    void transition_to(FollowerState new_state)
    {
        if (new_state == state_) return;
        RCLCPP_INFO(this->get_logger(), "FSM: %s → %s",
            state_to_str(state_).c_str(), state_to_str(new_state).c_str());
        state_ = new_state;

        std_msgs::msg::String s;
        s.data = state_to_str(state_);
        pub_state_->publish(s);
    }

    void publish_disabled_stop_once()
    {
        if (disabled_stop_published_) return;
        // Send a single brake command on disable without fighting other
        // base controllers with repeated zero twists.
        geometry_msgs::msg::Twist stop;
        pub_cmd_vel_->publish(stop);
        disabled_stop_published_ = true;
    }

    // Prediccion Kalman
    void predict(double dt)
    {
        F_(0, 1) = dt;
        F_(2, 3) = dt;
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // correccion Kalman
    void update(const Eigen::Vector2d& z)
    {
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
    }

    // calculo de velocidade
    geometry_msgs::msg::Twist calculate_speeds(double target_x, double target_y)
    {
        geometry_msgs::msg::Twist cmd;
        double distance   = std::hypot(target_x, target_y);
        double angle      = std::atan2(target_y, target_x);
        double error_dist = distance - dist_to_human_;

        double kalman_trace = P_.trace();
        double dynamic_deadband = 0.05 + std::min(0.15, kalman_trace * 0.05);

        if (std::abs(error_dist) < dynamic_deadband) {
            cmd.linear.x = 0.0;
        } else {
            cmd.linear.x = error_dist * control_alpha_;
        }

        if (!move_backwards_ && cmd.linear.x < 0.0) {
            cmd.linear.x = 0.0;
        }

        cmd.angular.z = angle * control_alpha_;

        if (std::abs(angle) > 0.15) {
            cmd.linear.x *= std::cos(angle * control_beta_);
        }

        cmd.linear.x  = std::clamp(cmd.linear.x,  -max_linear_,  max_linear_);
        cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_, max_angular_);
        return cmd;
    }

    void control_loop()
    {
        // Publicar diagnóstico de incertidumbre Kalman
        std_msgs::msg::Float32 trace_msg;
        trace_msg.data = static_cast<float>(P_.trace());
        pub_kalman_trace_->publish(trace_msg);

        if (!enable_) {
            publish_disabled_stop_once();
            last_linear_vel_  = 0.0;
            last_angular_vel_ = 0.0;
            transition_to(FollowerState::STOPPED);
            return;
        }

        rclcpp::Time now = this->get_clock()->now();
        double dt_predict = (now - last_time_).seconds();
        if (dt_predict <= 0.0) dt_predict = 0.05;

        predict(dt_predict);

        bool data_legs = (no_legs_ < 8  && current_legs_.header.stamp.nanosec > 0);
        bool data_body = (no_body_ < 15 && current_body_.header.stamp.nanosec > 0);

        if (data_legs || data_body)
        {
            transition_to(FollowerState::FOLLOWING);

            // normalizacion de pesos de fusion
            double w_legs = 0.0, w_body = 0.0;
            if (data_legs && data_body) {
                double total = leg_weight_ + body_weight_;
                w_legs = (total > 1e-6) ? leg_weight_ / total : 0.5;
                w_body = (total > 1e-6) ? body_weight_ / total : 0.5;
            } else if (data_legs) {
                w_legs = 1.0; w_body = 0.0;
            } else {
                w_legs = 0.0; w_body = 1.0;
            }

            double z_x = w_legs * current_legs_.point.x + w_body * current_body_.point.x;
            double z_y = w_legs * current_legs_.point.y + w_body * current_body_.point.y;

            double robot_speed  = std::hypot(last_linear_vel_, last_angular_vel_);
            double kalman_trace = P_.trace();
            double adaptive_gate = tracking_gate_
                                 + gate_vel_factor_   * robot_speed
                                 + gate_trace_factor_ * kalman_trace;

            double dist_gate = std::hypot(z_x - x_(0), z_y - x_(2));
            if (x_(0) == 0.0 && x_(2) == 0.0) dist_gate = 0.0;

            if (dist_gate < adaptive_gate) {
                Eigen::Vector2d z(z_x, z_y);
                update(z);
            }

            double filtered_x = x_(0);
            double filtered_y = x_(2);

            geometry_msgs::msg::PointStamped dbg;
            dbg.header.frame_id = "base_link";
            dbg.header.stamp    = now;
            dbg.point.x = filtered_x; dbg.point.y = filtered_y; dbg.point.z = 0.5;
            pub_debug_point_->publish(dbg);

            geometry_msgs::msg::Twist raw_cmd = calculate_speeds(filtered_x, filtered_y);

            // Ramp de aceleración
            double dt = (now - last_time_).seconds();
            if (dt > 0 && dt < 0.1) {
                double max_dv = MAX_ACCEL       * dt;
                double max_dw = MAX_ALPHA_ACCEL * dt;
                last_linear_vel_  += std::clamp(raw_cmd.linear.x  - last_linear_vel_,  -max_dv, max_dv);
                last_angular_vel_ += std::clamp(raw_cmd.angular.z - last_angular_vel_, -max_dw, max_dw);
            } else {
                last_linear_vel_  = raw_cmd.linear.x;
                last_angular_vel_ = raw_cmd.angular.z;
            }
            last_time_ = now;

            geometry_msgs::msg::Twist smooth_cmd;
            smooth_cmd.linear.x  = last_linear_vel_;
            smooth_cmd.angular.z = last_angular_vel_;
            pub_cmd_vel_->publish(smooth_cmd);

        }
        else
        {
            no_legs_++;
            no_body_++;
            last_time_ = now;

            if (no_legs_ > recovery_stop_frames_) {
                // Demasiado tiempo sin datos para completamente
                transition_to(FollowerState::STOPPED);
                last_linear_vel_  = 0.0;
                last_angular_vel_ = 0.0;
                geometry_msgs::msg::Twist stop;
                pub_cmd_vel_->publish(stop);

            } else if (no_legs_ > recovery_start_frames_) {
                // buscando al humano
                transition_to(FollowerState::RECOVERY);

                // Determinar direccion de búsqueda basada en última posición conocida
                if (x_(2) != 0.0) {
                    recovery_dir_ = (x_(2) > 0.0) ? 1.0 : -1.0;
                }

                geometry_msgs::msg::Twist recovery_cmd;
                recovery_cmd.linear.x  = 0.0;
                recovery_cmd.angular.z = recovery_dir_ * recovery_angular_vel_;
                pub_cmd_vel_->publish(recovery_cmd);

            } else {
                geometry_msgs::msg::Twist coasted;
                coasted.linear.x  = last_linear_vel_  * 0.8;
                coasted.angular.z = last_angular_vel_ * 0.8;
                last_linear_vel_  = coasted.linear.x;
                last_angular_vel_ = coasted.angular.z;
                pub_cmd_vel_->publish(coasted);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
