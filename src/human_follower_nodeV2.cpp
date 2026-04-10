#include <iostream>
#include <cmath>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// Includes para la fusión estable
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class HumanFollower : public rclcpp::Node
{
    // Política de sincronización: Piernas + Cuerpo
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PointStamped, 
        geometry_msgs::msg::PointStamped> SyncPolicy;

public:
    HumanFollower()
    : Node("human_follower"), enable_(false)
    {
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING STABLE HUMAN FOLLOWER ---");

        // Parámetros
        control_alpha_  = this->declare_parameter("control_alpha", 0.6548);
        control_beta_   = this->declare_parameter("control_beta", 0.3);
        max_linear_     = this->declare_parameter("max_linear", 0.3);
        max_angular_    = this->declare_parameter("max_angular", 0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.65);
        move_backwards_ = this->declare_parameter("move_backwards", false);
        cmd_vel_topic_   = this->declare_parameter("cmd_vel_topic", std::string("/hardware/base/cmd_vel"));
        legs_pose_topic_ = this->declare_parameter("legs_pose_topic", std::string("/hri/leg_finder/leg_pose"));
        person_pose_topic_ = this->declare_parameter("person_pose_topic", std::string("/human_detection/target_point2"));

        // TF2 setup
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        pub_debug_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/human_follower/target_debug", 10);
        
        // --- Setup de Fusión Estable (Message Filters) ---
        legs_sub_filter_.subscribe(this, legs_pose_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        body_sub_filter_.subscribe(this, person_pose_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), legs_sub_filter_, body_sub_filter_);
        sync_->registerCallback(std::bind(&HumanFollower::callback_fusion, this, std::placeholders::_1, std::placeholders::_2));
        
        // Subscriptions de control
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            std::bind(&HumanFollower::callback_enable, this, std::placeholders::_1));
        
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1,
            std::bind(&HumanFollower::callback_stop, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FUSION SYSTEM READY: Waiting for enable...");
    }

private:
    // Filtros y Sync
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> legs_sub_filter_;
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> body_sub_filter_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Pubs y Subs
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_point_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Params y Estado
    float control_alpha_, control_beta_, max_linear_, max_angular_, dist_to_human_;
    bool move_backwards_, enable_;
    bool use_potfields_ = false;
    std::string legs_pose_topic_, person_pose_topic_, cmd_vel_topic_;

    // Variables de Filtro LPF y Deadband
    float last_target_x_ = 0.0, last_target_y_ = 0.0;
    const float LPF_ALPHA = 0.35;  // Suavizado
    const float DEADBAND = 0.04;   // 4cm zona muerta
    
    // Constantes de Fusión
    const float MAX_STD_DEV = 0.6;    
    const float LEG_WEIGHT = 0.55;     
    const float BODY_WEIGHT = 0.45;    

    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "SYSTEM %s", enable_ ? "ENABLED" : "DISABLED");
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr) {
        enable_ = false;
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        RCLCPP_WARN(this->get_logger(), "STOP RECEIVED - Robot Halted");
    }

    // --- CALLBACK DE FUSIÓN (Reemplaza al control_loop) ---
    void callback_fusion(const geometry_msgs::msg::PointStamped::ConstSharedPtr& legs_msg,
                         const geometry_msgs::msg::PointStamped::ConstSharedPtr& body_msg)
    {
        if (!enable_) return;

        float lx, ly, bx, by;
        try {
            transform_to_robot_position(legs_msg->point.x, legs_msg->point.y, legs_msg->header.frame_id, lx, ly);
            transform_to_robot_position(body_msg->point.x, body_msg->point.y, body_msg->header.frame_id, bx, by);
        } catch (...) { return; }

        float dist_diff = sqrt(pow(lx - bx, 2) + pow(ly - by, 2));
        float target_x, target_y;

        // Lógica de Fusión
        if (dist_diff < MAX_STD_DEV) {
            target_x = (lx * LEG_WEIGHT) + (bx * BODY_WEIGHT);
            target_y = (ly * LEG_WEIGHT) + (by * BODY_WEIGHT);
        } else {
            // Si hay mucha discrepancia, confiamos en el cuerpo (Visión)
            target_x = bx; target_y = by;
        }

        // TÉCNICA: Deadband (Evitar vibraciones en estático)
        float move_step = sqrt(pow(target_x - last_target_x_, 2) + pow(target_y - last_target_y_, 2));
        if (move_step < DEADBAND) {
            target_x = last_target_x_;
            target_y = last_target_y_;
        }

        // TÉCNICA: Filtro Paso Bajo (LPF) para suavidad
        target_x = LPF_ALPHA * target_x + (1.0 - LPF_ALPHA) * last_target_x_;
        target_y = LPF_ALPHA * target_y + (1.0 - LPF_ALPHA) * last_target_y_;

        last_target_x_ = target_x;
        last_target_y_ = target_y;

        // Debug Point
        geometry_msgs::msg::PointStamped dbg;
        dbg.header.frame_id = "base_link";
        dbg.header.stamp = this->get_clock()->now();
        dbg.point.x = target_x; dbg.point.y = target_y; dbg.point.z = 0.5;
        pub_debug_point_->publish(dbg);

        // Control de Motores
        geometry_msgs::msg::Twist cmd = calculate_speeds(target_x, target_y);
        pub_cmd_vel_->publish(cmd);
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
            throw std::runtime_error("TF Failure"); 
        }
    }

    geometry_msgs::msg::Twist calculate_speeds(float gx, float gy) {
        float angle_error = atan2(gy, gx);
        float dist = sqrt(gx*gx + gy*gy) - dist_to_human_;
        
        if (!move_backwards_ && dist < 0) dist = 0;
        if (dist > max_linear_) dist = max_linear_;

        geometry_msgs::msg::Twist result;
        if (dist > 0.02 || (move_backwards_ && std::abs(dist) > 0.05)) {
            result.linear.x = dist * exp(-(angle_error * angle_error) / control_alpha_);
            result.angular.z = max_angular_ * (2 / (1 + exp(-angle_error / control_beta_)) - 1);
        }
        return result;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}