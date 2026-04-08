#include <iostream>
#include <cmath>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class HumanFollower : public rclcpp::Node
{
public:
    HumanFollower()
    : Node("human_follower"), enable_(false)
    {
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER ---");
        //
        // Parámetros
        control_alpha_  = this->declare_parameter("control_alpha", 0.6548);
        control_beta_   = this->declare_parameter("control_beta", 0.3);
        max_linear_     = this->declare_parameter("max_linear", 0.3);
        max_angular_    = this->declare_parameter("max_angular", 0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.65);
        move_backwards_ = this->declare_parameter("move_backwards", false);
        cmd_vel_topic_   = this->declare_parameter("cmd_vel_topic", std::string("/hardware/base/cmd_vel"));
        legs_pose_topic_ = this->declare_parameter("legs_pose_topic", std::string("/hri/leg_finder/leg_pose"));
        person_pose_topic_ = this->declare_parameter("person_pose_topic", std::string("/human_detection/target_point"));

        // TF2 setup
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
         pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 
            rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal));

        pub_debug_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/human_follower/target_debug", 
            rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal));
        
        // Subscriptions
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            std::bind(&HumanFollower::callback_enable, this, std::placeholders::_1));
        
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1,
            std::bind(&HumanFollower::callback_stop, this, std::placeholders::_1));

        sub_body_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            person_pose_topic_, 1,
            std::bind(&HumanFollower::callback_body_pose, this, std::placeholders::_1));

        
        control_timer_ = this->create_wall_timer(
            30ms, std::bind(&HumanFollower::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "FUSION MODE: Legs + Person Tracker Active");
    }

private:
    // Pub/Sub y Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_body_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_point_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Params y Estado
    float control_alpha_, control_beta_, max_linear_, max_angular_, dist_to_human_;
    bool move_backwards_, enable_, use_potfields_;
    std::string legs_pose_topic_, person_pose_topic_, cmd_vel_topic_;

    //potenciales
    float repulsiveForce_=0.0f;
    const float KInfRep_ = 0.03f;
    
    //edad de las lecturas
    geometry_msgs::msg::PointStamped last_legs_msg_, last_body_msg_;
    bool have_legs_ = false, have_body_ = false;
    int no_legs_ = 0;

    // Constantes de Fusión
    const float MAX_STD_DEV = 0.65;    
    const float MAX_AGE_MS = 1000.0;   
    const float LEG_WEIGHT = 0.55;     
    const float BODY_WEIGHT = 0.45;  
    
    //filtro Kalman
    struct KalmanFilter {

        float x = 0.0f; // Estado estimado inicial (en metros).
        float p = 1.0f; // Error de la estimación inicial. Un valor alto indica que no confiamos en el valor inicial de 'x'.
        float q = 0.05f; // Ruido de proceso: Menor valor = más suavidad (filtro más lento)
        float r = 0.18f; // Ruido de medición: Mayor valor = ignora más el ruido del sensor

        float filter(float measurement){
            p = p + q;
            float k = p / (p + r);
            x = x + k * (measurement - x);
            p = (1.0f - k) * p;
            return x;
        }
    };
    
    KalmanFilter kalman_x, kalman_y;

    //suavidar velocidades
    double last_linear_vel_ = 0.0;  // Almacena la última velocidad lineal enviada
    double last_angular_vel_ = 0.0; // Almacena la última velocidad de giro
    double last_valid_x_ = 0.0;    // Coordenada X de la última posición filtrada del humano
    double last_valid_y_ = 0.0;    // Coordenada Y de la última posición filtrada del humano
    rclcpp::Time last_time_;
    const double MAX_ACCEL = 0.40; // aceleracion lienal
    const double MAX_ALPHA_ACCEL = 0.85; // angular

    const float TRACKING_GATE = 0.45; // metros perm. por salto de lecturas

    // --- Callbacks ---
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
        if (enable_) {
            RCLCPP_INFO(this->get_logger(), "SYSTEM ENABLED - Resetting stamps");
            last_legs_msg_.header.stamp = this->get_clock()->now();
            last_body_msg_.header.stamp = this->get_clock()->now();
            have_legs_ = false;
            have_body_ = false;
            last_time_ = this->get_clock()->now();

            auto sensor_qos = rclcpp::QoS(10).best_effort().durability_volatile();

            sub_legs_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                legs_pose_topic_, sensor_qos,
                std::bind(&HumanFollower::callback_legs_pose, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "SYSTEM DISABLED");
            sub_legs_pose_.reset();
            sub_body_pose_.reset();
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr) {
        sub_legs_pose_.reset();
        sub_body_pose_.reset();
        enable_ = false;
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        RCLCPP_WARN(this->get_logger(), "STOP RECEIVED");
    }

    void callback_legs_pose(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_legs_msg_ = *msg;
        last_legs_msg_.header.stamp = this->get_clock()->now();
        have_legs_ = true;
    }

    void callback_body_pose(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_body_msg_ = *msg;
        have_body_ = true;
    }

    void transform_to_robot_position(float x, float y, const std::string &frame_id,
                                     float &x_out, float &y_out)
    {
        try {
            auto tf = tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero, 50ms);
            Eigen::Affine3d e = tf2::transformToEigen(tf.transform);
            Eigen::Vector3d v = e * Eigen::Vector3d(x, y, 0);
            x_out = v.x(); y_out = v.y();
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Error [%s]: %s", frame_id.c_str(), ex.what());
            throw; 
        }
    }

    // --- Lógica de Control ---
    geometry_msgs::msg::Twist calculate_speeds(float gx, float gy) {
        geometry_msgs::msg::Twist result;
        float angle_error = atan2(gy, gx);
        float dist_total = sqrt(gx*gx + gy*gy);
        float dist_error = dist_total - dist_to_human_;

        float target_lineal = dist_error;
        
        if (!move_backwards_ && target_lineal < 0) target_lineal = 0.0f;
        if (target_lineal > max_linear_) target_lineal = max_linear_;
        if (target_lineal < -max_linear_) target_lineal = -max_linear_;

        const float D_DIST = 0.05f;
        const float D_ANGLE = 0.05f;

        if (std::abs(dist_error) > D_DIST || std::abs(angle_error) > D_ANGLE) {
            result.linear.x = target_lineal * exp(-(angle_error * angle_error) / control_alpha_);
            result.linear.y = 0.0f;
            if (use_potfields_) result.linear.y = KInfRep_ * repulsiveForce_;
            result.angular.z = max_angular_ * (2.0f / (1.0f + exp(-angle_error / control_beta_)) - 1.0f);
        }
        else {
            result.linear.x = 0.0f;
            result.linear.y = 0.0f;
            if (fabs(angle_error) >= M_PI_4 / 6.0f)
                result.angular.z = max_angular_ * (2.0f / (1.0f + exp(-angle_error / control_beta_)) - 1.0f);
            else
                result.angular.z = 0.0f;
        }
        return result;
    }

    void control_loop() {
        if (!enable_) {
            //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for enable (true) on /hri/human_following/enable");
            return;
        }

        auto now = this->get_clock()->now();
        float final_x = 0.0, final_y = 0.0;
        bool target_valid = false;

        // 1. Validar tiempos
        double legs_age = (now - last_legs_msg_.header.stamp).seconds() * 1000.0;
        double body_age = (now - last_body_msg_.header.stamp).seconds() * 1000.0;

        bool legs_ok = have_legs_ && legs_age < MAX_AGE_MS;
        bool body_ok = have_body_ && body_age < MAX_AGE_MS;

        // 2. Transformar
        float lx, ly, bx, by;
        bool tf_legs_success = false;
        bool tf_body_success = false;

        try {
            if (legs_ok) {
                transform_to_robot_position(last_legs_msg_.point.x, last_legs_msg_.point.y, last_legs_msg_.header.frame_id, lx, ly);
                tf_legs_success = true;
            }
            if (body_ok) {
                transform_to_robot_position(last_body_msg_.point.x, last_body_msg_.point.y, last_body_msg_.header.frame_id, bx, by);
                tf_body_success = true;
            }
        } catch (...) { 
        }

        // 3. Fusión
        if (tf_legs_success && tf_body_success) {
            float dist = sqrt(pow(lx - bx, 2) + pow(ly - by, 2));
            if (dist < MAX_STD_DEV) {
                final_x = (lx * LEG_WEIGHT) + (bx * BODY_WEIGHT);
                final_y = (ly * LEG_WEIGHT) + (by * BODY_WEIGHT);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FUSION SUCCESS: Dist: %.2f | Age L:%.0fms B:%.0fms", dist, legs_age, body_age);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FUSION REJECTED: Dist %.2f > %.2f | Using Body", dist, MAX_STD_DEV);
                final_x = bx; final_y = by;
            }
            target_valid = true;
        } 
        else if (tf_legs_success) { 
            final_x = lx; final_y = ly; target_valid = true; 
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ONLY LEGS (Body Age: %.0fms, OK: %d)", body_age, body_ok); 
        }
        else if (tf_body_success) { 
            final_x = bx; final_y = by; target_valid = true; 
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ONLY BODY (Legs Age: %.0fms, OK: %d)", legs_age, legs_ok); 
        }
        else {
             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "NO TARGET: LegsAge: %.0fms BodyAge: %.0fms", legs_age, body_age);
        }

        // Validación de saltos (Gating)
         if (target_valid) {
            float dist_jump = sqrt(pow(final_x - last_valid_x_, 2) + pow(final_y - last_valid_y_, 2));
            if (last_valid_x_ != 0 && dist_jump > TRACKING_GATE) {
                //RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jump detect");
                target_valid = false;
            }
        }

        if (target_valid) {
            float filtered_x = kalman_x.filter(final_x);
            float filtered_y = kalman_y.filter(final_y);
            last_valid_x_ = filtered_x;
            last_valid_y_ = filtered_y;
            no_legs_ = 0;

            geometry_msgs::msg::PointStamped dbg;
            dbg.header.frame_id = "base_link"; dbg.header.stamp = now;
            dbg.point.x = filtered_x; dbg.point.y = filtered_y; dbg.point.z = 0.5;
            pub_debug_point_->publish(dbg);

            // Rampa de aceleración
            geometry_msgs::msg::Twist raw_cmd = calculate_speeds(filtered_x, filtered_y);
            
            double dt = (now - last_time_).seconds();
            if (dt > 0 && dt < 0.1) { 
                double max_dv = MAX_ACCEL * dt;
                double dv = raw_cmd.linear.x - last_linear_vel_;
                last_linear_vel_ += std::clamp(dv, -max_dv, max_dv);

                double max_dw = MAX_ALPHA_ACCEL * dt;
                double dw = raw_cmd.angular.z - last_angular_vel_;
                last_angular_vel_ += std::clamp(dw, -max_dw, max_dw);
            } else {
                last_linear_vel_ = raw_cmd.linear.x;
                last_angular_vel_ = raw_cmd.angular.z;
            }
            last_time_ = now;

            geometry_msgs::msg::Twist smooth_cmd;
            smooth_cmd.linear.x = last_linear_vel_;
            smooth_cmd.angular.z = last_angular_vel_;
            pub_cmd_vel_->publish(smooth_cmd);
            
        } 
        else { 
            if (++no_legs_ > 20) {
                last_linear_vel_ = 0.0;
                last_angular_vel_ = 0.0;
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
