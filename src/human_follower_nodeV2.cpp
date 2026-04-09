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
<<<<<<< HEAD
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING STABLE HUMAN FOLLOWER ---");

=======
        RCLCPP_INFO(this->get_logger(), "--- INITIALIZING HUMAN FOLLOWER ---");
        //
>>>>>>> 07956fa8d2c4ee5a25f0cbe40ef1bd1002df4c38
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
<<<<<<< HEAD
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        pub_debug_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/human_follower/target_debug", 10);
=======
         pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 
            rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal));

        pub_debug_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/human_follower/target_debug", 
            rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal));
>>>>>>> 07956fa8d2c4ee5a25f0cbe40ef1bd1002df4c38
        
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
    
<<<<<<< HEAD
    // Constantes de Fusión
    const float MAX_STD_DEV = 0.6;    
    const float LEG_WEIGHT = 0.55;     
    const float BODY_WEIGHT = 0.45;    
=======
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
>>>>>>> 07956fa8d2c4ee5a25f0cbe40ef1bd1002df4c38

    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        enable_ = msg->data;
<<<<<<< HEAD
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
=======
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
>>>>>>> 07956fa8d2c4ee5a25f0cbe40ef1bd1002df4c38

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
<<<<<<< HEAD
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
=======
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
>>>>>>> 07956fa8d2c4ee5a25f0cbe40ef1bd1002df4c38
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanFollower>());
    rclcpp::shutdown();
    return 0;
}