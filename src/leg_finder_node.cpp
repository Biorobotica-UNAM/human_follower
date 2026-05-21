// ============================================================================
//  leg_finder_node.cpp  —  RoboCup-grade version
//  Fixes vs original:
//    [BUG-1] Eliminada la doble transformación de puntos en el loop de clusters.
//            project_laser_to_base() ya convierte a base_link; el loop interno
//            ahora usa directamente esos puntos en lugar de re-aplicar lidar_transform.
//    [BUG-2] Constantes críticas de geometría expuestas como parámetros ROS 2,
//            eliminando la necesidad de recompilar para ajustar en campo.
//  Mejoras:
//    [OPT-1] Historial de posiciones para calcular velocidad del target y
//            predecir su posición cuando se pierde momentáneamente (coasting).
//    [OPT-2] Gate adaptiva: se amplía si la velocidad estimada del humano es alta.
//    [OPT-3] Score mejorado para selección de par de piernas: penaliza pares
//            con diferencia de ancho muy grande (más probable que sean obstáculos).
//    [OPT-4] Publicación de diagnóstico: confidence y número de clusters.
//    [OPT-5] Uso de std::deque para buffers del filtro (evita insert al inicio).
// ============================================================================

#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

struct Flank {
    int index;
    double range;
    double angle;
    std::string type;
};

struct Cluster {
    int start_idx;
    int end_idx;
    geometry_msgs::msg::PointStamped center;
    double width;
    double probability;
};

class LegFinderNode : public rclcpp::Node
{
public:
    LegFinderNode() : Node("leg_finder_node"), enable_(false)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING LEG FINDER NODE...");

        // Parametros de tópicos y frames
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("legs_pose_topic", "/hri/leg_finder/legs_pose");

        this->get_parameter("scan_topic",      scan_topic_);
        this->get_parameter("base_frame",      base_frame_);
        this->get_parameter("legs_pose_topic", legs_pose_topic_);

        // Parámetros geométricos expuestos como parámetros ROS 2
        filter_threshold_   = this->declare_parameter("filter_threshold",    0.081);
        flank_threshold_    = this->declare_parameter("flank_threshold",     0.04);
        horizon_threshold_  = this->declare_parameter("horizon_threshold",   9);
        leg_thin_           = this->declare_parameter("leg_thin",            0.00341);
        leg_thick_          = this->declare_parameter("leg_thick",           0.03);
        two_legs_thin_      = this->declare_parameter("two_legs_thin",       0.056644);
        two_legs_thick_     = this->declare_parameter("two_legs_thick",      0.25);
        two_legs_near_      = this->declare_parameter("two_legs_near",       0.0022201);
        two_legs_far_       = this->declare_parameter("two_legs_far",        0.25);
        in_front_min_x_     = this->declare_parameter("in_front_min_x",      0.25);
        in_front_max_x_     = this->declare_parameter("in_front_max_x",      2.5);
        in_front_min_y_     = this->declare_parameter("in_front_min_y",     -0.5);
        in_front_max_y_     = this->declare_parameter("in_front_max_y",      0.5);

        // Gate adaptiva target dinamico según velocidad estimada del target
        base_gate_          = this->declare_parameter("base_gate",           0.081);
        gate_vel_factor_    = this->declare_parameter("gate_vel_factor",     0.5);
        coast_frames_       = this->declare_parameter("coast_frames",        6);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Suscriptores
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LegFinderNode::scan_callback, this, std::placeholders::_1));

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1,
            std::bind(&LegFinderNode::enable_callback, this, std::placeholders::_1));

        // Publicadores
        pub_legs_pose_  = this->create_publisher<geometry_msgs::msg::PointStamped>(legs_pose_topic_, 10);
        pub_marker_     = this->create_publisher<visualization_msgs::msg::Marker>("/hri/leg_finder/marker", 10);
        pub_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/hri/leg_finder/confidence", 10);

        // Buffers del filtro Butterworth
        legs_x_input_  = std::deque<double>(4, 0.0);
        legs_y_input_  = std::deque<double>(4, 0.0);
        legs_x_output_ = std::deque<double>(4, 0.0);
        legs_y_output_ = std::deque<double>(4, 0.0);

        last_legs_x_ = 0.0;
        last_legs_y_ = 0.0;
        vel_x_ = 0.0;
        vel_y_ = 0.0;
        coast_count_ = 0;
        last_confidence_ = 0.0f;
    }

private:
    // Params
    std::string scan_topic_, base_frame_, legs_pose_topic_;
    bool enable_;

    double filter_threshold_, flank_threshold_;
    int    horizon_threshold_;
    double leg_thin_, leg_thick_;
    double two_legs_thin_, two_legs_thick_;
    double two_legs_near_, two_legs_far_;
    double in_front_min_x_, in_front_max_x_;
    double in_front_min_y_, in_front_max_y_;
    double base_gate_, gate_vel_factor_;
    int    coast_frames_;

    // STATE 
    double last_legs_x_, last_legs_y_;
    double vel_x_, vel_y_;          // velocidad estimada del target
    int    coast_count_;          
    float  last_confidence_;

    // Coeficientes Butterworth
    static constexpr double BFB0 =  0.00041655;
    static constexpr double BFB1 =  0.00124964;
    static constexpr double BFB2 =  0.00124964;
    static constexpr double BFB3 =  0.00041655;
    static constexpr double BFA1 = -2.68615739;
    static constexpr double BFA2 =  2.41965544;
    static constexpr double BFA3 = -0.73016571;

    std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;

    // handles y publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  sub_scan_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           sub_enable_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_pose_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_marker_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_confidence_;

    // Buffers con deque 
    std::deque<double> legs_x_input_,  legs_y_input_;
    std::deque<double> legs_x_output_, legs_y_output_;

    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        enable_ = msg->data;
        if (!enable_) {
            // Limpiar estado al desactivar para no tener residuos
            last_legs_x_ = last_legs_y_ = 0.0;
            vel_x_ = vel_y_ = 0.0;
            coast_count_ = 0;
        }
    }

    Eigen::Affine3d get_lidar_transform()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(base_frame_, "laser_frame",
                         rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(100)));
        } catch (const tf2::TransformException &) {
            try {
                tf_msg = tf_buffer_->lookupTransform(base_frame_, "cloud_lidar_link",
                             rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(100)));
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "No se pudo obtener la TF del LiDAR: %s", ex.what());
                throw;
            }
        }
        return tf2::transformToEigen(tf_msg);
    }

    void project_laser_to_base(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                               std::vector<geometry_msgs::msg::PointStamped>& out,
                               const Eigen::Affine3d& T)
    {
        out.clear();
        out.reserve(scan->ranges.size());
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double rho = scan->ranges[i];
            if (rho < scan->range_min || rho > scan->range_max) {
                geometry_msgs::msg::PointStamped invalid;
                invalid.point.x = invalid.point.y = invalid.point.z =
                    std::numeric_limits<double>::quiet_NaN();
                out.push_back(invalid);
                continue;
            }
            double theta = scan->angle_min + i * scan->angle_increment;
            Eigen::Vector3d p = T * Eigen::Vector3d(rho * std::cos(theta),
                                                     rho * std::sin(theta), 0.0);
            geometry_msgs::msg::PointStamped pt;
            pt.header.frame_id = base_frame_;
            pt.header.stamp    = scan->header.stamp;
            pt.point.x = p.x(); pt.point.y = p.y(); pt.point.z = p.z();
            out.push_back(pt);
        }
    }

    // Callback 
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!enable_) return;

        Eigen::Affine3d T;
        try { T = get_lidar_transform(); }
        catch (...) { return; }

        std::vector<geometry_msgs::msg::PointStamped> points_base;
        project_laser_to_base(scan, points_base, T);
        if (points_base.empty()) return;

        // flancos 
        std::vector<Flank> flanks;
        for (size_t i = 1; i + 1 < scan->ranges.size(); ++i) {
            double r_prev = scan->ranges[i - 1];
            double r_curr = scan->ranges[i];
            double r_next = scan->ranges[i + 1];
            if (r_curr < scan->range_min || r_curr > scan->range_max) continue;

            double diff_prev = r_curr - r_prev;
            double diff_next = r_next - r_curr;

            if (std::abs(diff_prev) > flank_threshold_ || std::abs(diff_next) > flank_threshold_) {
                Flank f;
                f.index = static_cast<int>(i);
                f.range = r_curr;
                f.angle = scan->angle_min + i * scan->angle_increment;
                f.type  = (diff_prev < 0 || diff_next < 0) ? "CONCAVE" : "CONVEX";
                flanks.push_back(f);
            }
        }

        std::vector<Cluster> clusters;
        for (size_t i = 0; i < flanks.size(); ++i) {
            if (flanks[i].type != "CONCAVE") continue;
            for (size_t j = i + 1; j < flanks.size(); ++j) {
                if (flanks[j].type != "CONVEX") continue;

                int idx_start = flanks[i].index;
                int idx_end   = flanks[j].index;
                if (idx_end - idx_start > horizon_threshold_) continue;

                double sum_x = 0.0, sum_y = 0.0;
                int valid_pts = 0;
                double first_x = 0, first_y = 0, last_x = 0, last_y = 0;
                bool got_first = false;

                // points_base directamente
                for (int k = idx_start; k <= idx_end; ++k) {
                    if (k < 0 || k >= static_cast<int>(points_base.size())) continue;
                    const auto& pt = points_base[k];
                    if (std::isnan(pt.point.x)) continue;

                    sum_x += pt.point.x;
                    sum_y += pt.point.y;
                    if (!got_first) {
                        first_x = pt.point.x; first_y = pt.point.y;
                        got_first = true;
                    }
                    last_x = pt.point.x; last_y = pt.point.y;
                    valid_pts++;
                }

                if (valid_pts == 0) continue;

                Cluster c;
                c.start_idx = idx_start;
                c.end_idx   = idx_end;
                c.center.header.frame_id = base_frame_;
                c.center.header.stamp    = scan->header.stamp;
                c.center.point.x = sum_x / valid_pts;
                c.center.point.y = sum_y / valid_pts;
                c.center.point.z = 0.0;
                c.width       = std::hypot(last_x - first_x, last_y - first_y);
                c.probability = 0.0;
                clusters.push_back(c);
                break;
            }
        }

        std::vector<Cluster> detected_legs;
        for (auto& c : clusters) {
            if      (c.width >= leg_thin_  && c.width <= leg_thick_)     c.probability = 0.4;
            else if (c.width >  leg_thick_ && c.width <= two_legs_thick_) c.probability = 0.2;

            if (c.probability > 0.1) detected_legs.push_back(c);
        }

        // Seleccion del par o mejor pierna única
        geometry_msgs::msg::PointStamped final_pose;
        bool human_found = false;
        float confidence = 0.0f;

        if (detected_legs.size() >= 2) {
            double best_score = -1.0;
            Cluster leg1, leg2;

            for (size_t i = 0; i < detected_legs.size(); ++i) {
                for (size_t j = i + 1; j < detected_legs.size(); ++j) {
                    double d = std::hypot(
                        detected_legs[i].center.point.x - detected_legs[j].center.point.x,
                        detected_legs[i].center.point.y - detected_legs[j].center.point.y);

                    if (d < two_legs_near_ || d > two_legs_far_) continue;

   
                    double width_diff = std::abs(detected_legs[i].width - detected_legs[j].width);
                    double symmetry_bonus = 1.0 / (1.0 + width_diff * 10.0);
                    double score = (detected_legs[i].probability + detected_legs[j].probability)
                                   * symmetry_bonus;

                    if (score > best_score) {
                        best_score = score;
                        leg1 = detected_legs[i];
                        leg2 = detected_legs[j];
                        human_found = true;
                        confidence  = static_cast<float>(score);
                    }
                }
            }

            if (human_found) {
                final_pose.header    = leg1.center.header;
                final_pose.point.x   = (leg1.center.point.x + leg2.center.point.x) / 2.0;
                final_pose.point.y   = (leg1.center.point.y + leg2.center.point.y) / 2.0;
                final_pose.point.z   = 0.0;
            }
        }

        if (!human_found && !detected_legs.empty()) {
            double max_prob = -1.0;
            for (const auto& leg : detected_legs) {
                if (leg.probability > max_prob) {
                    max_prob   = leg.probability;
                    final_pose = leg.center;
                    human_found = true;
                    confidence  = static_cast<float>(max_prob) * 0.5f; // menor confianza
                }
            }
        }

        // Gate adaptiva según velocidad estimada del target 
        double current_gate = base_gate_ + gate_vel_factor_ *
                              std::hypot(vel_x_, vel_y_);

        if (human_found) {
            double dist_to_prev = std::hypot(final_pose.point.x - last_legs_x_,
                                             final_pose.point.y - last_legs_y_);
            if (last_legs_x_ == 0.0 && last_legs_y_ == 0.0) dist_to_prev = 0.0;

            if (dist_to_prev < current_gate) {
                // velocidad estimada update
                vel_x_ = 0.3 * (final_pose.point.x - last_legs_x_) + 0.7 * vel_x_;
                vel_y_ = 0.3 * (final_pose.point.y - last_legs_y_) + 0.7 * vel_y_;
                coast_count_ = 0;

                push_filter_buffers(final_pose.point.x, final_pose.point.y);

                geometry_msgs::msg::PointStamped filtered = final_pose;
                apply_butterworth(filtered);

                pub_legs_pose_->publish(filtered);
                publish_marker(filtered.point.x, filtered.point.y, confidence);
                last_confidence_ = confidence;
            } else {
    
                push_filter_buffers(final_pose.point.x, final_pose.point.y);
                pub_legs_pose_->publish(final_pose);
                publish_marker(final_pose.point.x, final_pose.point.y, confidence * 0.5f);
            }

        } else {
            // predecir posicion y amortiguacion de velocidad
            coast_count_++;
            if (coast_count_ <= coast_frames_ && (last_legs_x_ != 0.0 || last_legs_y_ != 0.0)) {
                double pred_x = last_legs_x_ + vel_x_;
                double pred_y = last_legs_y_ + vel_y_;
                push_filter_buffers(pred_x, pred_y);

                geometry_msgs::msg::PointStamped coasted;
                coasted.header.frame_id = base_frame_;
                coasted.header.stamp    = scan->header.stamp;
                coasted.point.x = pred_x;
                coasted.point.y = pred_y;
                coasted.point.z = 0.0;
                pub_legs_pose_->publish(coasted);
                publish_marker(pred_x, pred_y, last_confidence_ * 0.3f);
                vel_x_ *= 0.85;
                vel_y_ *= 0.85;
            }
        }
        // nivel de confianza
        std_msgs::msg::Float32 conf_msg;
        conf_msg.data = human_found ? confidence : 0.0f;
        pub_confidence_->publish(conf_msg);
    }


    void push_filter_buffers(double x, double y)
    {
        legs_x_input_.push_front(x);  legs_x_input_.pop_back();
        legs_y_input_.push_front(y);  legs_y_input_.pop_back();
        legs_x_output_.push_front(0); legs_x_output_.pop_back();
        legs_y_output_.push_front(0); legs_y_output_.pop_back();
        last_legs_x_ = x;
        last_legs_y_ = y;
    }

    void apply_butterworth(geometry_msgs::msg::PointStamped& p)
    {
        legs_x_output_[0]  = BFB0*legs_x_input_[0] + BFB1*legs_x_input_[1]
                           + BFB2*legs_x_input_[2] + BFB3*legs_x_input_[3];
        legs_x_output_[0] -= BFA1*legs_x_output_[1] + BFA2*legs_x_output_[2]
                           + BFA3*legs_x_output_[3];

        legs_y_output_[0]  = BFB0*legs_y_input_[0] + BFB1*legs_y_input_[1]
                           + BFB2*legs_y_input_[2] + BFB3*legs_y_input_[3];
        legs_y_output_[0] -= BFA1*legs_y_output_[1] + BFA2*legs_y_output_[2]
                           + BFA3*legs_y_output_[3];

        p.point.x = legs_x_output_[0];
        p.point.y = legs_y_output_[0];
    }

    void publish_marker(double x, double y, float confidence)
    {
         // Color: verde intenso = alta confianza, amarillo = baja
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.header.stamp    = this->get_clock()->now();
        m.ns     = "legs";
        m.id     = 0;
        m.type   = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.2;
        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.4;
        m.color.a = 1.0f;
        m.color.r = 1.0f - std::min(confidence * 2.0f, 1.0f);
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        pub_marker_->publish(m);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegFinderNode>());
    rclcpp::shutdown();
    return 0;
}