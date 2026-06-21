
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
    HumanFollower() : Node("human_follower"), enable_(false), kalman_initialized_(false),
                      state_(FollowerState::STOPPED)
    {
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER ---");

        dist_to_human_ = this->declare_parameter("distance_to_human", 0.70);   // más distancia = fuera del costmap inflado
        leg_weight_    = this->declare_parameter("fusion.leg_weight",  0.55);
        body_weight_   = this->declare_parameter("fusion.body_weight", 0.45);
        tracking_gate_ = this->declare_parameter("fusion.tracking_gate", 2.0); 
        goal_topic_    = this->declare_parameter<std::string>("goal_topic", "/nav_control/goal");
        stop_topic_    = this->declare_parameter<std::string>("stop_topic", "/navigation/stop");
        global_frame_  = this->declare_parameter<std::string>("global_frame", "map");
        goal_min_dist_ = this->declare_parameter("goal_min_dist", 0.15); //  15-0.08 calibrar para la fluidez que se necesite

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

        pub_goal_  = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        pub_stop_  = this->create_publisher<std_msgs::msg::Empty>(stop_topic_, 10);
        pub_state_ = this->create_publisher<std_msgs::msg::String>("/hri/human_follower/state", 10);
        pub_debug_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
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
        last_goal_x_ = last_goal_y_ = 0.0;

        timer_ = this->create_wall_timer(200ms, std::bind(&HumanFollower::control_loop, this)); // mas reactividad con 100-200ms 
        RCLCPP_INFO(this->get_logger(), "HumanFollower ready. Goal → %s  stop → %s  dist=%.2f  gate=%.2f",
            goal_topic_.c_str(), stop_topic_.c_str(), dist_to_human_, tracking_gate_);
    }

private:
    bool enable_;
    bool kalman_initialized_;
    FollowerState state_;
    double dist_to_human_, leg_weight_, body_weight_, tracking_gate_, goal_min_dist_;
    std::string goal_topic_, stop_topic_, global_frame_;
    int no_legs_, no_body_;
    rclcpp::Time last_time_;
    double last_goal_x_, last_goal_y_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              sub_enable_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_body_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pub_goal_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                pub_stop_;
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
            pub_stop_->publish(std_msgs::msg::Empty{});
            RCLCPP_INFO(this->get_logger(), "HumanFollower disabled; published navigation stop");
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
        last_goal_x_ = last_goal_y_ = 0.0;
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

    void control_loop()
    {
        if (!enable_) return;

        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 2.0) dt = 0.5;
        last_time_ = now;

        bool have_legs = (no_legs_ < 4  && current_legs_.header.stamp.nanosec > 0);
        bool have_body = (no_body_ < 6  && current_body_.header.stamp.nanosec > 0);

        if (!have_legs && !have_body) {
            no_legs_++; no_body_++;
            if (no_legs_ > 10) { transition_to(FollowerState::STOPPED); kalman_reset(); }
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
            // Primera medición: inicializar directamente sin predecir
            x_(0) = zx; x_(2) = zy;
            x_(1) = x_(3) = 0.0;
            kalman_initialized_ = true;
        } else {
            // Ciclos normales: predecir solo cuando hay datos activos
            kalman_predict(dt);
            double dist_gate = std::hypot(zx - x_(0), zy - x_(2));
            if (dist_gate < tracking_gate_) kalman_update(zx, zy);
            else {
                // Medición muy lejos del estado — reinicializar en lugar de ignorar
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Gate excedida (%.2f > %.2f) — reinicializando Kalman", dist_gate, tracking_gate_);
                x_(0) = zx; x_(2) = zy; x_(1) = x_(3) = 0.0;
                P_ = Eigen::Matrix4d::Identity() * 1.0;
            }
        }

        double fx = x_(0), fy = x_(2);

        geometry_msgs::msg::PointStamped dbg;
        dbg.header.frame_id = "base_link"; dbg.header.stamp = now;
        dbg.point.x = fx; dbg.point.y = fy; dbg.point.z = 0.5;
        pub_debug_->publish(dbg);

        double angle     = std::atan2(fy, fx);
        double dist      = std::hypot(fx, fy);
        double goal_dist = std::max(0.0, dist - dist_to_human_);

        if (dist <= dist_to_human_ + 0.1) { no_legs_++; no_body_++; return; }

        geometry_msgs::msg::PoseStamped goal_base;
        goal_base.header.frame_id = "base_link";
        goal_base.header.stamp    = now;
        goal_base.pose.position.x = goal_dist * std::cos(angle);
        goal_base.pose.position.y = goal_dist * std::sin(angle);
        goal_base.pose.position.z = 0.0;
        goal_base.pose.orientation.z = std::sin(angle / 2.0);
        goal_base.pose.orientation.w = std::cos(angle / 2.0);

        try {
            auto goal_map = tf_buffer_->transform(goal_base, global_frame_,
                                tf2::durationFromSec(0.1));
            double delta = std::hypot(goal_map.pose.position.x - last_goal_x_,
                                      goal_map.pose.position.y - last_goal_y_);
            if (delta >= goal_min_dist_) {
                pub_goal_->publish(goal_map);
                last_goal_x_ = goal_map.pose.position.x;
                last_goal_y_ = goal_map.pose.position.y;
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF error: %s", ex.what());
        }

        no_legs_++; no_body_++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}
