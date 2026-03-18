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
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER (DEBUG MODE) ---");

        // Parámetros
        control_alpha_  = this->declare_parameter("control_alpha", 0.6548);
        control_beta_   = this->declare_parameter("control_beta", 0.3);
        max_linear_     = this->declare_parameter("max_linear", 0.3);
        max_angular_    = this->declare_parameter("max_angular", 0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.9);
        move_backwards_ = this->declare_parameter("move_backwards", false);
        cmd_vel_topic_   = this->declare_parameter("cmd_vel_topic", std::string("/hardware/base/cmd_vel"));
        legs_pose_topic_ = this->declare_parameter("legs_pose_topic", std::string("/hri/leg_finder/leg_pose"));
        person_pose_topic_ = this->declare_parameter("person_pose_topic", std::string("/human_detection/target_point"));

        // TF2 setup
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        pub_debug_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/human_follower/target_debug", 10);
        
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
    bool move_backwards_, enable_;
    std::string legs_pose_topic_, person_pose_topic_, cmd_vel_topic_;
    
    geometry_msgs::msg::PointStamped last_legs_msg_, last_body_msg_;
    bool have_legs_ = false, have_body_ = false;
    int no_legs_ = 0;

    // Constantes de Fusión
    const float MAX_STD_DEV = 0.6;    //distancia maxima aceptable legs/body
    const float MAX_AGE_MS = 600.0;   // tiempo maximo de validez de un mensaje
    const float LEG_WEIGHT = 0.6;     //influencia de las piernas 
    const float BODY_WEIGHT = 0.4;    //influencia del cuerpo

    // --- Callbacks ---
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
        if (enable_) {
            RCLCPP_INFO(this->get_logger(), "SYSTEM ENABLED");
            sub_legs_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                legs_pose_topic_, 1,
                std::bind(&HumanFollower::callback_legs_pose, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "SYSTEM DISABLED");
            sub_legs_pose_.reset();
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr) {
        enable_ = false;
        RCLCPP_WARN(this->get_logger(), "STOP RECEIVED");
    }

    void callback_legs_pose(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_legs_msg_ = *msg;
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
            auto tf = tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
            Eigen::Affine3d e = tf2::transformToEigen(tf.transform);
            Eigen::Vector3d v = e * Eigen::Vector3d(x, y, 0);
            x_out = v.x(); y_out = v.y();
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Error: %s", ex.what());
            throw; 
        }
    }

    void control_loop() {
        if (!enable_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for enable (true) on /hri/human_following/enable");
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
        try {
            if (legs_ok) transform_to_robot_position(last_legs_msg_.point.x, last_legs_msg_.point.y, last_legs_msg_.header.frame_id, lx, ly);
            if (body_ok) transform_to_robot_position(last_body_msg_.point.x, last_body_msg_.point.y, last_body_msg_.header.frame_id, bx, by);
        } catch (...) { return; }

        // 3. Fusión
        if (legs_ok && body_ok) {
            float dist = sqrt(pow(lx - bx, 2) + pow(ly - by, 2));
            if (dist < MAX_STD_DEV) {
                final_x = (lx * LEG_WEIGHT) + (bx * BODY_WEIGHT);
                final_y = (ly * LEG_WEIGHT) + (by * BODY_WEIGHT);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FUSION: Legs + Body");
            } else {
                final_x = bx; final_y = by;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "DISCREPANCY: Using Body");
            }
            target_valid = true;
        } 
        else if (legs_ok) { final_x = lx; final_y = ly; target_valid = true; RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Only Legs"); }
        else if (body_ok) { final_x = bx; final_y = by; target_valid = true; RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Only Body"); }
        else {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 9000, "NO DATA (Age L: %.0fms, B: %.0fms)", legs_age, body_age);
        }

        if (target_valid) {
            no_legs_ = 0;
            // Publicar punto de Debug
            geometry_msgs::msg::PointStamped dbg;
            dbg.header.frame_id = "base_link"; dbg.header.stamp = now;
            dbg.point.x = final_x; dbg.point.y = final_y; dbg.point.z = 0.5;
            pub_debug_point_->publish(dbg);

            geometry_msgs::msg::Twist cmd = calculate_speeds(final_x, final_y);
            pub_cmd_vel_->publish(cmd);
        } else {
            if (++no_legs_ > 20) {
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            }
        }
    }

    geometry_msgs::msg::Twist calculate_speeds(float gx, float gy) {
        float angle_error = atan2(gy, gx);
        float dist = sqrt(gx*gx + gy*gy) - dist_to_human_;
        
        if (!move_backwards_ && dist < 0) dist = 0;
        if (dist > max_linear_) dist = max_linear_;

        geometry_msgs::msg::Twist res;
        if (dist > 0 || (move_backwards_ && std::abs(dist) > 0.05)) {
            res.linear.x = dist * exp(-(angle_error * angle_error) / control_alpha_);
            res.angular.z = max_angular_ * (2 / (1 + exp(-angle_error / control_beta_)) - 1);
        }
        return res;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
