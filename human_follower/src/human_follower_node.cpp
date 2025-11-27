#include <iostream>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

class HumanFollower : public rclcpp::Node
{
public:
    HumanFollower()
    : Node("human_follower"), enable_(false), new_legs_pose_(false)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING HUMAN FOLLOWER NODE...");

        // Parámetros
        control_alpha_  = this->declare_parameter("control_alpha", 0.6548);
        control_beta_   = this->declare_parameter("control_beta", 0.3);
        max_linear_     = this->declare_parameter("max_linear", 0.3);
        max_angular_    = this->declare_parameter("max_angular", 0.7);
        dist_to_human_  = this->declare_parameter("distance_to_human", 0.9);
        move_backwards_ = this->declare_parameter("move_backwards", false);
        use_potfields_  = this->declare_parameter("enable_potfields", false);
        cmd_vel_topic_   = this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
        legs_pose_topic_ = this->declare_parameter("legs_pose_topic", std::string("/hri/leg_finder/leg_pose"));

        // TF2 setup
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Pub/Sub
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1,
            std::bind(&HumanFollower::callback_enable, this, std::placeholders::_1));
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1,
            std::bind(&HumanFollower::callback_stop, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), 
            "max_linear=%.2f max_angular=%.2f alpha=%.2f beta=%.2f",
            max_linear_, max_angular_, control_alpha_, control_beta_);
        RCLCPP_INFO(this->get_logger(), 
            "cmd_vel topic: %s  legs_pose_topic: %s",
            cmd_vel_topic_.c_str(), legs_pose_topic_.c_str());
    }

    void run()
    {
        rclcpp::Rate rate(30); // 30 Hz
        int no_legs = 0;

        while (rclcpp::ok())
        {
            if (enable_)
            {
                if (new_legs_pose_)
                    no_legs = 0;
                else
                {
                    if (++no_legs > 30)
                    {
                        no_legs = 0;
                        geometry_msgs::msg::Twist cmd;
                        cmd.linear.x = 0.0;
                        cmd.angular.z = 0.0;
                        pub_cmd_vel_->publish(cmd);
                    }
                }
                new_legs_pose_ = false;
            }

            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

private:
    // Subs / pubs
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_legs_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parámetros
    float control_alpha_, control_beta_, max_linear_, max_angular_, dist_to_human_;
    bool move_backwards_, use_potfields_;
    std::string legs_pose_topic_, cmd_vel_topic_;

    // Estado interno
    bool enable_, new_legs_pose_;

    // Potenciales
    float repulsiveForce_ = 0.0f;
    float KInfRep_ = 0.03f;

    // --- Callbacks ---
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        enable_ = msg->data;
        if (enable_)
        {
            sub_legs_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                legs_pose_topic_, 1,
                std::bind(&HumanFollower::callback_legs_pose, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Enabled human follower");
            }
        else
        {
            sub_legs_pose_.reset();
            RCLCPP_INFO(this->get_logger(), "Disabled human follower");
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr)
    {
        sub_legs_pose_.reset();
        enable_ = false;
        RCLCPP_INFO(this->get_logger(), "Stopped human follower");
    }

    void callback_legs_pose(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        float human_x = msg->point.x;
        float human_y = msg->point.y;

        if (msg->header.frame_id != "base_link")
        {
            try {
                transform_to_robot_position(human_x, human_y, msg->header.frame_id, human_x, human_y);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 error: %s", ex.what());
                return;
            }
        }

        pub_cmd_vel_->publish(calculate_speeds(human_x, human_y));
        new_legs_pose_ = true;
    }

    geometry_msgs::msg::Twist calculate_speeds(float goal_x, float goal_y)
    {
        float angle_error = atan2(goal_y, goal_x);
        float distance = sqrt(goal_x * goal_x + goal_y * goal_y);
        distance -= dist_to_human_;
        if (!move_backwards_ && distance < 0) distance = 0;
        if (distance > max_linear_) distance = max_linear_;

        geometry_msgs::msg::Twist result;
        if (distance > 0 || move_backwards_)
        {
            result.linear.x = distance * exp(-(angle_error * angle_error) / control_alpha_);
            result.linear.y = 0;
            if (use_potfields_) result.linear.y = KInfRep_ * repulsiveForce_;
            result.angular.z = max_angular_ * (2 / (1 + exp(-angle_error / control_beta_)) - 1);
        }
        else
        {
            result.linear.x = 0;
            result.linear.y = 0;
            if (fabs(angle_error) >= M_PI_4 / 6.0f)
                result.angular.z = max_angular_ * (2 / (1 + exp(-angle_error / control_beta_)) - 1);
            else
                result.angular.z = 0;
        }
        return result;
    }

    void transform_to_robot_position(float x, float y, const std::string &frame_id,
                                     float &x_wrt_robot, float &y_wrt_robot)
    {
        geometry_msgs::msg::TransformStamped tf_msg =
            tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
        Eigen::Affine3d e = tf2::transformToEigen(tf_msg.transform);
        Eigen::Vector3d v(x, y, 0);
        v = e * v;
        x_wrt_robot = v.x();
        y_wrt_robot = v.y();
    }
};

// --- main ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanFollower>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
