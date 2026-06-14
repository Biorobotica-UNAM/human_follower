#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;

enum class FollowerState { STOPPED, FOLLOWING };

class HumanFollower : public rclcpp::Node
{
public:
    HumanFollower() : Node("human_follower"), enable_(false), state_(FollowerState::STOPPED),
                      kalman_initialized_(false)
    {
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER ---");

        dist_to_human_ = this->declare_parameter("distance_to_human", 0.70);
        leg_weight_    = this->declare_parameter("fusion.leg_weight",  0.55);
        body_weight_   = this->declare_parameter("fusion.body_weight", 0.45);
        tracking_gate_ = this->declare_parameter("fusion.tracking_gate", 2.0);
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/hardware/base/cmd_vel");


        max_linear_vel_  = this->declare_parameter("max_linear_vel",  0.4);   // m/s
        max_angular_vel_ = this->declare_parameter("max_angular_vel", 0.6);   // rad/s
        linear_accel_    = this->declare_parameter("linear_accel",  0.08);  // 0.0-1.0, menor = más suave
        angular_accel_   = this->declare_parameter("angular_accel", 0.12);  // 0.0-1.0

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            std::bind(&HumanFollower::cb_enable, this, std::placeholders::_1));
        sub_legs_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/hri/leg_finder/legs_pose", 1,
            std::bind(&HumanFollower::cb_legs, this, std::placeholders::_1));
        sub_body_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/human_detection/target_point", 1,
            std::bind(&HumanFollower::cb_body, this, std::placeholders::_1));

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        pub_state_   = this->create_publisher<std_msgs::msg::String>("/hri/human_follower/state", 10);
        pub_debug_   = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/hri/human_follower/debug_fused_point", 10);

        // Kalman 4-estados: [x, vx, y, vy]
        x_ = Eigen::Vector4d::Zero();
        P_ = Eigen::Matrix4d::Identity() * 1.0;
        F_ = Eigen::Matrix4d::Identity();
        H_ = Eigen::MatrixXd::Zero(2, 4);
        H_(0,0) = 1.0; H_(1,2) = 1.0;
        Q_ = Eigen::Matrix4d::Identity() * 0.01;
        R_ = Eigen::Matrix2d::Identity() * 0.1;

        last_time_ = this->get_clock()->now();
        no_legs_ = no_body_ = 0;
        current_linear_vel_  = 0.0;
        current_angular_vel_ = 0.0;

        timer_ = this->create_wall_timer(200ms, std::bind(&HumanFollower::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "HumanFollower ready. cmd_vel → %s  dist=%.2f  vmax=%.2f",
            cmd_vel_topic_.c_str(), dist_to_human_, max_linear_vel_);
    }

private:
    bool enable_;
    bool kalman_initialized_;
    FollowerState state_;
    double dist_to_human_, leg_weight_, body_weight_, tracking_gate_;
    double max_linear_vel_, max_angular_vel_, linear_accel_, angular_accel_;
    double current_linear_vel_, current_angular_vel_;
    std::string cmd_vel_topic_;
    int no_legs_, no_body_;
    rclcpp::Time last_time_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              sub_enable_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_body_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               pub_state_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr    pub_debug_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    Eigen::Vector4d  x_;
    Eigen::Matrix4d  P_, F_, Q_;
    Eigen::MatrixXd  H_;
    Eigen::Matrix2d  R_;

    geometry_msgs::msg::PointStamped current_legs_, current_body_;

    void cb_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
        if (!enable_) {
            transition_to(FollowerState::STOPPED);
            kalman_reset();
            publish_stop();
        } else {
            RCLCPP_INFO(this->get_logger(), "HumanFollower RE-ACTIVADO — buscando target...");
        }
    }
    void cb_legs(const geometry_msgs::msg::PointStamped::SharedPtr msg) { current_legs_=*msg; no_legs_=0; }
    void cb_body(const geometry_msgs::msg::PointStamped::SharedPtr msg) { current_body_=*msg; no_body_=0; }

    void transition_to(FollowerState s) {
        if (s == state_) return;
        const char* names[] = {"STOPPED","FOLLOWING"};
        RCLCPP_INFO(this->get_logger(), "FSM: %s → %s", names[(int)state_], names[(int)s]);
        state_ = s;
        std_msgs::msg::String msg; msg.data = names[(int)s];
        pub_state_->publish(msg);
    }

    void kalman_reset() {
        x_ = Eigen::Vector4d::Zero();
        P_ = Eigen::Matrix4d::Identity() * 1.0;
        kalman_initialized_ = false;
    }

    void kalman_predict(double dt) {
        F_(0,1) = dt; F_(2,3) = dt;
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void kalman_update(double zx, double zy) {
        Eigen::Vector2d z(zx, zy);
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
    }

    void publish_stop() {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = stop.angular.z = 0.0;
        pub_cmd_vel_->publish(stop);
        current_linear_vel_  = 0.0;
        current_angular_vel_ = 0.0;
    }

    double ramp(double current, double target, double alpha) {
        return current + alpha * (target - current);
    }

    void control_loop()
    {
        if (!enable_) return;

        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 2.0) dt = 0.2;
        last_time_ = now;

        bool have_legs = (no_legs_ < 4  && current_legs_.header.stamp.nanosec > 0);
        bool have_body = (no_body_ < 6  && current_body_.header.stamp.nanosec > 0);

        if (!have_legs && !have_body) {
            no_legs_++; no_body_++;
            if (no_legs_ > 10) {
                transition_to(FollowerState::STOPPED);
                kalman_reset();
                publish_stop();
            } else {
                current_linear_vel_  = ramp(current_linear_vel_,  0.0, linear_accel_);
                current_angular_vel_ = ramp(current_angular_vel_, 0.0, angular_accel_);
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x  = current_linear_vel_;
                cmd.angular.z = current_angular_vel_;
                pub_cmd_vel_->publish(cmd);
            }
            return;
        }

        transition_to(FollowerState::FOLLOWING);

        // Fusión con pesos normalizados
        double wl=0, wb=0;
        if (have_legs && have_body) {
            double t = leg_weight_ + body_weight_;
            wl = leg_weight_/t; wb = body_weight_/t;
        } else if (have_legs) { wl=1.0; }
        else                  { wb=1.0; }

        double zx = wl*current_legs_.point.x + wb*current_body_.point.x;
        double zy = wl*current_legs_.point.y + wb*current_body_.point.y;

        if (!kalman_initialized_) {
            x_(0) = zx; x_(2) = zy;
            x_(1) = x_(3) = 0.0;
            kalman_initialized_ = true;
        } else {
            kalman_predict(dt);
            double dist_gate = std::hypot(zx - x_(0), zy - x_(2));
            if (dist_gate < tracking_gate_) {
                kalman_update(zx, zy);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Gate excedida (%.2f > %.2f) — reinicializando Kalman", dist_gate, tracking_gate_);
                x_(0) = zx; x_(2) = zy; x_(1) = x_(3) = 0.0;
                P_ = Eigen::Matrix4d::Identity() * 1.0;
            }
        }

        double fx = x_(0), fy = x_(2);

        // Debug
        geometry_msgs::msg::PointStamped dbg;
        dbg.header.frame_id = "base_link"; dbg.header.stamp = now;
        dbg.point.x = fx; dbg.point.y = fy; dbg.point.z = 0.5;
        pub_debug_->publish(dbg);

        double dist  = std::hypot(fx, fy);
        double angle = std::atan2(fy, fx);
        double linear_error = dist - dist_to_human_;
        double target_linear = std::clamp(linear_error * 0.5, -max_linear_vel_, max_linear_vel_);
        double target_angular = std::clamp(angle * 1.2, -max_angular_vel_, max_angular_vel_);

        if (std::abs(linear_error) < 0.05) target_linear = 0.0;
        current_linear_vel_  = ramp(current_linear_vel_,  target_linear,  linear_accel_);
        current_angular_vel_ = ramp(current_angular_vel_, target_angular, angular_accel_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = current_linear_vel_;
        cmd.angular.z = current_angular_vel_;
        pub_cmd_vel_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}