#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
<<<<<<< HEAD

#include "rclcpp/rclcpp.hpp"
=======
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
>>>>>>> 269b512 (human follower shit)
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

<<<<<<< HEAD
// Mantenemos tus constantes geométricas (son muy precisas)
#define FILTER_THRESHOLD  .081
#define FLANK_THRESHOLD  .04
#define HORIZON_THRESHOLD  9
#define LEG_THIN  0.00341
#define LEG_THICK  0.03
#define TWO_LEGS_THIN  0.056644
#define TWO_LEGS_THICK  0.25
#define TWO_LEGS_NEAR  0.0022201
#define TWO_LEGS_FAR  0.25
#define IS_LEG_THRESHOLD 0.5
#define MAX_FLOAT 1e6

struct Flank {
=======
// ====== CONSTANTES GEOMÉTRICAS ======
#define FLANK_THRESHOLD    0.04
#define HORIZON_THRESHOLD  9.0
#define LEG_THIN           0.00341
#define LEG_THICK          0.03
#define TWO_LEGS_THIN      0.056644
#define TWO_LEGS_THICK     0.25
#define TWO_LEGS_NEAR      0.0022201
#define TWO_LEGS_FAR       0.25
#define MAX_FLOAT          1e6

// ====== BUTTERWORTH ======
#define BFA0X 1.0
#define BFA1X -1.760041880343169
#define BFA2X 1.182893262037831
#define BFA3X -0.278059917634546
#define BFB0X 0.018098933007514
#define BFB1X 0.054296799022543
#define BFB2X 0.054296799022543
#define BFB3X 0.018098933007514

#define BFA0Y 1.0
#define BFA1Y -1.760041880343169
#define BFA2Y 1.182893262037831
#define BFA3Y -0.278059917634546
#define BFB0Y 0.018098933007514
#define BFB1Y 0.054296799022543
#define BFB2Y 0.054296799022543
#define BFB3Y 0.018098933007514

struct Flank
{
>>>>>>> 269b512 (human follower shit)
    float x;
    float y;
    bool used = false;
};

<<<<<<< HEAD
class LegFinder : public rclcpp::Node {
public:
    LegFinder() : Node("leg_finder"), set_stage_(true) {
        // Parámetros
        laser_scan_topic_ = this->declare_parameter("laser_scan_topic", "/scan");
        frame_id_         = this->declare_parameter("frame_id", "base_link");
        
        // Publishers
        pub_legs_pose_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hri/leg_finder/leg_pose", 1);
        pub_legs_found_ = this->create_publisher<std_msgs::msg::Bool>("/hri/leg_finder/leg_detected", 1);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/hri/leg_finder/hypothesis", 1);

        // Subscriber (Siempre activo o gestionado por el Tracker via enable)
        sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LegFinder::callback_scan, this, std::placeholders::_1));
=======
struct Candidate
{
    float x = 0.0f;
    float y = 0.0f;
    float dist_pred = MAX_FLOAT;
    float score = MAX_FLOAT;
    bool valid = false;
};

class LegFinder : public rclcpp::Node
{
public:
    LegFinder()
    : Node("leg_finder"),
      set_stage_(true),
      enabled_(false),
      legs_found_(false),
      stop_robot_(false),
      acquisition_counter_(0),
      lost_counter_(0),
      stable_match_count_(0),
      track_x_(0.0f),
      track_y_(0.0f),
      vel_x_(0.0f),
      vel_y_(0.0f),
      track_confidence_(0.0f),
      preferred_x_(0.0f),
      preferred_y_(0.0f),
      have_preferred_candidate_(false)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING LEG FINDER V3");

        laser_scan_topic_   = this->declare_parameter<std::string>("laser_scan_topic", "/scan");
        frame_id_           = this->declare_parameter<std::string>("frame_id", "base_link");
        show_hypothesis_    = this->declare_parameter<bool>("show_hypothesis", true);

        // Ventana de adquisición
        in_front_min_x_     = this->declare_parameter<double>("in_front_min_x", 0.25);
        in_front_max_x_     = this->declare_parameter<double>("in_front_max_x", 2.5);
        in_front_min_y_     = this->declare_parameter<double>("in_front_min_y", -0.5);
        in_front_max_y_     = this->declare_parameter<double>("in_front_max_y", 0.5);

        // Límites globales del tracking
        max_tracking_x_     = this->declare_parameter<double>("max_tracking_x", 3.5);
        max_abs_tracking_y_ = this->declare_parameter<double>("max_abs_tracking_y", 1.3);

        acquisition_frames_ = this->declare_parameter<int>("acquisition_frames", 6);
        max_lost_frames_    = this->declare_parameter<int>("max_lost_frames", 18);

        // Gating
        strong_gate_near_   = this->declare_parameter<double>("strong_gate_near", 0.10);
        strong_gate_far_    = this->declare_parameter<double>("strong_gate_far", 0.20);
        weak_gate_near_     = this->declare_parameter<double>("weak_gate_near", 0.22);
        weak_gate_far_      = this->declare_parameter<double>("weak_gate_far", 0.38);
        gate_distance_scale_= this->declare_parameter<double>("gate_distance_scale", 0.08);

        max_lateral_jump_   = this->declare_parameter<double>("max_lateral_jump", 0.22);
        max_velocity_       = this->declare_parameter<double>("max_velocity", 0.8);

        // Suavizado
        velocity_alpha_     = this->declare_parameter<double>("velocity_alpha", 0.20);
        measurement_alpha_  = this->declare_parameter<double>("measurement_alpha", 0.22);

        // Rechazo de saltos
        stationary_jump_reject_ = this->declare_parameter<double>("stationary_jump_reject", 0.08);
        moving_jump_reject_     = this->declare_parameter<double>("moving_jump_reject", 0.20);
        stationary_speed_thr_   = this->declare_parameter<double>("stationary_speed_thr", 0.08);

        // Histéresis / confianza
        confidence_gain_strong_ = this->declare_parameter<double>("confidence_gain_strong", 0.12);
        confidence_gain_weak_   = this->declare_parameter<double>("confidence_gain_weak", 0.03);
        confidence_decay_weak_  = this->declare_parameter<double>("confidence_decay_weak", 0.02);
        confidence_decay_lost_  = this->declare_parameter<double>("confidence_decay_lost", 0.08);
        min_conf_to_keep_track_ = this->declare_parameter<double>("min_conf_to_keep_track", 0.20);

        switch_margin_          = this->declare_parameter<double>("switch_margin", 0.08);
        switch_confirm_frames_  = this->declare_parameter<int>("switch_confirm_frames", 4);

        pub_legs_hypothesis_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/hri/leg_finder/hypothesis", 10);
        pub_legs_pose_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/hri/leg_finder/leg_pose", 10);
        pub_legs_found_ = this->create_publisher<std_msgs::msg::Bool>(
            "/hri/leg_finder/legs_found", 10);

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1,
            std::bind(&LegFinder::callback_enable, this, std::placeholders::_1));

        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1,
            std::bind(&LegFinder::callback_stop, this, std::placeholders::_1));
>>>>>>> 269b512 (human follower shit)

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

<<<<<<< HEAD
        RCLCPP_INFO(this->get_logger(), "Leg Finder Simplificado (Sensor Mode) iniciado.");
    }

private:
    // ... (Variables de TF y tópicos similares al anterior) ...
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_legs_found_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    
    std::string laser_scan_topic_, frame_id_, laser_scan_frame_;
    bool set_stage_;

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (set_stage_) {
            laser_scan_frame_ = msg->header.frame_id;
            set_stage_ = false;
        }

        std::vector<float> legs_x, legs_y;
        find_leg_hypothesis(msg, legs_x, legs_y);

        // Publicar marcador para RVIZ
        publish_marker(legs_x, legs_y);

        // Lógica: Buscar la más cercana al frente del robot (Sector de búsqueda)
        float target_x = MAX_FLOAT, target_y = MAX_FLOAT;
        bool found = get_best_candidate(legs_x, legs_y, target_x, target_y);

        // Reportar al Tracker
        std_msgs::msg::Bool found_msg;
        found_msg.data = found;
        pub_legs_found_->publish(found_msg);

        if (found) {
            geometry_msgs::msg::PointStamped pose_msg;
            pose_msg.header.stamp = msg->header.stamp; // Usar el tiempo del láser original
            pose_msg.header.frame_id = frame_id_;
            pose_msg.point.x = target_x;
            pose_msg.point.y = target_y;
            pose_msg.point.z = 0.0;
            pub_legs_pose_->publish(pose_msg);
        }
    }

    // --- Mantenemos tus funciones de geometría (detect_flanks, etc) ---
    // PERO: Quitamos el filtro Butterworth y el acumulador de 20 frames.

    bool is_leg(float x1, float y1, float x2, float y2) {
        float dx = x1 - x2;
        float dy = y1 - y2;
        float dist2 = dx*dx + dy*dy;
        return (dist2 > LEG_THIN && dist2 < LEG_THICK) ||
               (dist2 > TWO_LEGS_THIN && dist2 < TWO_LEGS_THICK);
    }

    bool get_best_candidate(std::vector<float>& lx, std::vector<float>& ly, float& tx, float& ty) {
        float min_dist = MAX_FLOAT;
        bool found = false;

        for (size_t i = 0; i < lx.size(); i++) {
            // Filtro de área de interés (ej: solo frente al robot hasta 3m)
            if (lx[i] > 0.2 && lx[i] < 3.5 && std::abs(ly[i]) < 1.5) {
                float d = sqrt(lx[i]*lx[i] + ly[i]*ly[i]);
                if (d < min_dist) {
                    min_dist = d;
                    tx = lx[i];
                    ty = ly[i];
                    found = true;
                }
            }
        }
        return found;
=======
        reset_filters(0.0f, 0.0f);
        last_stamp_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Leg finder V3 ready.");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_legs_hypothesis_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_legs_found_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool set_stage_;
    bool enabled_;
    bool legs_found_;
    bool stop_robot_;

    int acquisition_counter_;
    int lost_counter_;
    int stable_match_count_;

    float track_x_;
    float track_y_;
    float vel_x_;
    float vel_y_;

    float track_confidence_;

    float preferred_x_;
    float preferred_y_;
    bool have_preferred_candidate_;

    std::vector<float> x_in_;
    std::vector<float> x_out_;
    std::vector<float> y_in_;
    std::vector<float> y_out_;

    rclcpp::Time last_stamp_;

    std::string frame_id_;
    std::string laser_scan_topic_;
    std::string laser_scan_frame_;
    bool show_hypothesis_;

    double in_front_min_x_;
    double in_front_max_x_;
    double in_front_min_y_;
    double in_front_max_y_;

    double max_tracking_x_;
    double max_abs_tracking_y_;

    int acquisition_frames_;
    int max_lost_frames_;

    double strong_gate_near_;
    double strong_gate_far_;
    double weak_gate_near_;
    double weak_gate_far_;
    double gate_distance_scale_;

    double max_lateral_jump_;
    double max_velocity_;

    double velocity_alpha_;
    double measurement_alpha_;

    double stationary_jump_reject_;
    double moving_jump_reject_;
    double stationary_speed_thr_;

    double confidence_gain_strong_;
    double confidence_gain_weak_;
    double confidence_decay_weak_;
    double confidence_decay_lost_;
    double min_conf_to_keep_track_;

    double switch_margin_;
    int switch_confirm_frames_;

    // =========================
    // Callbacks
    // =========================
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        enabled_ = msg->data;

        if (enabled_) {
            if (!sub_laser_) {
                sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    laser_scan_topic_,
                    rclcpp::SensorDataQoS(),
                    std::bind(&LegFinder::callback_scan, this, std::placeholders::_1));
            }
            stop_robot_ = false;
            RCLCPP_INFO(this->get_logger(), "Enabled leg finder");
        } else {
            sub_laser_.reset();
            clear_tracking_state();
            RCLCPP_INFO(this->get_logger(), "Disabled leg finder");
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr)
    {
        stop_robot_ = true;
        sub_laser_.reset();
        clear_tracking_state();
        RCLCPP_INFO(this->get_logger(), "Stopped leg finder");
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!enabled_ || stop_robot_) {
            publish_found_flag(false);
            return;
        }

        if (set_stage_) {
            laser_scan_frame_ = msg->header.frame_id;
            set_stage_ = false;
            RCLCPP_INFO(this->get_logger(), "Laser scan frame set to %s", laser_scan_frame_.c_str());
        }

        std::vector<float> legs_x;
        std::vector<float> legs_y;

        try {
            find_leg_hypothesis(msg, legs_x, legs_y);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "TF error in leg_finder: %s", ex.what());
            publish_found_flag(false);
            return;
        }

        if (show_hypothesis_) {
            pub_legs_hypothesis_->publish(get_hypothesis_marker(legs_x, legs_y));
        }

        if (!legs_found_) {
            handle_acquisition(msg, legs_x, legs_y);
        } else {
            handle_tracking(msg, legs_x, legs_y);
        }

        publish_found_flag(legs_found_);
    }

    // =========================
    // Estado
    // =========================
    void clear_tracking_state()
    {
        legs_found_ = false;
        acquisition_counter_ = 0;
        lost_counter_ = 0;
        stable_match_count_ = 0;

        track_x_ = 0.0f;
        track_y_ = 0.0f;
        vel_x_ = 0.0f;
        vel_y_ = 0.0f;
        track_confidence_ = 0.0f;

        preferred_x_ = 0.0f;
        preferred_y_ = 0.0f;
        have_preferred_candidate_ = false;

        reset_filters(0.0f, 0.0f);
        last_stamp_ = this->now();
    }

    void publish_found_flag(bool value)
    {
        std_msgs::msg::Bool msg;
        msg.data = value;
        pub_legs_found_->publish(msg);
    }

    void reset_filters(float x, float y)
    {
        x_in_.assign(4, x);
        x_out_.assign(4, x);
        y_in_.assign(4, y);
        y_out_.assign(4, y);
    }

    // =========================
    // Utilidades
    // =========================
    bool valid_range(float r, const sensor_msgs::msg::LaserScan::SharedPtr & laser) const
    {
        return std::isfinite(r) && r >= laser->range_min && r <= laser->range_max;
    }

    bool inside_front_window(float x, float y) const
    {
        return x > in_front_min_x_ && x < in_front_max_x_ &&
               y > in_front_min_y_ && y < in_front_max_y_;
    }

    bool inside_tracking_window(float x, float y) const
    {
        return x > 0.10 && x < max_tracking_x_ &&
               std::fabs(y) < max_abs_tracking_y_;
    }

    float sqdist(float x1, float y1, float x2, float y2) const
    {
        float dx = x1 - x2;
        float dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    float adaptive_gate(float pred_x, float near_gate, float far_gate) const
    {
        float g = static_cast<float>(near_gate + gate_distance_scale_ * std::fabs(pred_x));
        return std::clamp(g, static_cast<float>(near_gate), static_cast<float>(far_gate));
    }

    // =========================
    // Geometría de piernas
    // =========================
    bool is_leg_span(float x1, float y1, float x2, float y2)
    {
        float d2 = sqdist(x1, y1, x2, y2);
        return (d2 > LEG_THIN && d2 < LEG_THICK) ||
               (d2 > TWO_LEGS_THIN && d2 < TWO_LEGS_THICK);
>>>>>>> 269b512 (human follower shit)
    }

    Eigen::Affine3d get_lidar_position()
    {
<<<<<<< HEAD
        geometry_msgs::msg::TransformStamped tf;
        tf = tf_buffer_->lookupTransform("base_link", laser_scan_frame_, tf2::TimePointZero);
=======
        geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(frame_id_, laser_scan_frame_, tf2::TimePointZero);
>>>>>>> 269b512 (human follower shit)
        return tf2::transformToEigen(tf.transform);
    }

    std::pair<std::vector<float>, std::vector<float>>
    project_laser_to_base(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
<<<<<<< HEAD
        std::vector<float> laser_x(laser->ranges.size());
        std::vector<float> laser_y(laser->ranges.size());

        Eigen::Affine3d lidar_to_robot = get_lidar_position();

        for (size_t i = 0; i < laser->ranges.size(); i++)
        {
            float theta = laser->angle_min + i * laser->angle_increment * scan_downsampling_;
            Eigen::Vector3d v(laser->ranges[i] * cos(theta),
                            laser->ranges[i] * sin(theta), 0.0);

            v = lidar_to_robot * v;

            laser_x[i] = v.x();
            laser_y[i] = v.y();
=======
        std::vector<float> laser_x(laser->ranges.size(),
                                   std::numeric_limits<float>::quiet_NaN());
        std::vector<float> laser_y(laser->ranges.size(),
                                   std::numeric_limits<float>::quiet_NaN());

        Eigen::Affine3d lidar_to_robot = get_lidar_position();

        for (size_t i = 0; i < laser->ranges.size(); i++) {
            float r = laser->ranges[i];
            if (!valid_range(r, laser))
                continue;

            float theta = laser->angle_min + static_cast<float>(i) * laser->angle_increment;
            Eigen::Vector3d v(r * std::cos(theta), r * std::sin(theta), 0.0);
            v = lidar_to_robot * v;

            laser_x[i] = static_cast<float>(v.x());
            laser_y[i] = static_cast<float>(v.y());
>>>>>>> 269b512 (human follower shit)
        }

        return {laser_x, laser_y};
    }

    std::vector<Flank> detect_flanks(
        const sensor_msgs::msg::LaserScan::SharedPtr laser,
        const std::vector<float>& laser_x,
        const std::vector<float>& laser_y)
    {
        std::vector<Flank> flanks;
<<<<<<< HEAD
        size_t ant2 = 0;

        for (size_t i = 1; i < laser->ranges.size(); i++)
        {
            size_t ant = ant2;

            if (fabs(laser->ranges[i] - laser->ranges[i-1]) > FLANK_THRESHOLD)
                ant2 = i;

            if (fabs(laser->ranges[i] - laser->ranges[i-1]) > FLANK_THRESHOLD &&
                (is_leg(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) ||
                is_leg(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2])))
            {
                float dx = laser_x[ant] - laser_x[i-1];
                float dy = laser_y[ant] - laser_y[i-1];
                float dist2 = dx*dx + dy*dy;

                float sum_x = 0;
                float sum_y = 0;
                for (size_t j = ant; j < i; j++)
                {
                    sum_x += laser_x[j];
                    sum_y += laser_y[j];
                }

                float cx = sum_x / (i - ant);
                float cy = sum_y / (i - ant);

                if (dist2 > LEG_THIN && dist2 < LEG_THICK)
                {
                    flanks.push_back({cx, cy, false});
                }
                else if (dist2 > TWO_LEGS_THIN && dist2 < TWO_LEGS_THICK)
                {
                    flanks.push_back({cx, cy, true}); // ya considerada doble
                }
            }
=======
        size_t segment_start = 0;
        bool in_segment = false;

        for (size_t i = 1; i < laser->ranges.size(); i++) {
            float r_prev = laser->ranges[i - 1];
            float r_curr = laser->ranges[i];

            bool prev_valid = valid_range(r_prev, laser);
            bool curr_valid = valid_range(r_curr, laser);

            if (!prev_valid || !curr_valid) {
                in_segment = false;
                continue;
            }

            if (!in_segment) {
                segment_start = i - 1;
                in_segment = true;
            }

            bool jump = std::fabs(r_curr - r_prev) > FLANK_THRESHOLD;
            if (!jump)
                continue;

            size_t segment_end = i - 1;

            if (segment_end > segment_start &&
                std::isfinite(laser_x[segment_start]) && std::isfinite(laser_y[segment_start]) &&
                std::isfinite(laser_x[segment_end])   && std::isfinite(laser_y[segment_end])) {

                float d2 = sqdist(laser_x[segment_start], laser_y[segment_start],
                                  laser_x[segment_end], laser_y[segment_end]);

                if (is_leg_span(laser_x[segment_start], laser_y[segment_start],
                                laser_x[segment_end], laser_y[segment_end])) {

                    float sum_x = 0.0f;
                    float sum_y = 0.0f;
                    int count = 0;

                    for (size_t j = segment_start; j <= segment_end; j++) {
                        if (!std::isfinite(laser_x[j]) || !std::isfinite(laser_y[j]))
                            continue;
                        sum_x += laser_x[j];
                        sum_y += laser_y[j];
                        count++;
                    }

                    if (count > 0) {
                        float cx = sum_x / static_cast<float>(count);
                        float cy = sum_y / static_cast<float>(count);

                        if (d2 > LEG_THIN && d2 < LEG_THICK)
                            flanks.push_back({cx, cy, false});
                        else if (d2 > TWO_LEGS_THIN && d2 < TWO_LEGS_THICK)
                            flanks.push_back({cx, cy, true});
                    }
                }
            }

            segment_start = i;
            in_segment = true;
>>>>>>> 269b512 (human follower shit)
        }

        return flanks;
    }

    void detect_double_legs(std::vector<Flank>& flanks,
<<<<<<< HEAD
                        std::vector<float>& legs_x,
                        std::vector<float>& legs_y)
    {
        for (size_t i = 0; i < flanks.size() - 1; i++)
        {
            for (size_t j = i + 1; j < flanks.size(); j++)
            {
                if (flanks[i].used || flanks[j].used)
                    continue;

                float dx = flanks[i].x - flanks[j].x;
                float dy = flanks[i].y - flanks[j].y;
                float dist2 = dx*dx + dy*dy;

                if (dist2 > TWO_LEGS_NEAR && dist2 < TWO_LEGS_FAR)
                {
                    float px = (flanks[i].x + flanks[j].x) / 2.0;
                    float py = (flanks[i].y + flanks[j].y) / 2.0;

                    if ((px*px + py*py) < HORIZON_THRESHOLD)
                    {
=======
                            std::vector<float>& legs_x,
                            std::vector<float>& legs_y)
    {
        if (flanks.size() < 2)
            return;

        for (size_t i = 0; i + 1 < flanks.size(); i++) {
            for (size_t j = i + 1; j < flanks.size(); j++) {
                if (flanks[i].used || flanks[j].used)
                    continue;

                float d2 = sqdist(flanks[i].x, flanks[i].y, flanks[j].x, flanks[j].y);

                if (d2 > TWO_LEGS_NEAR && d2 < TWO_LEGS_FAR) {
                    float px = 0.5f * (flanks[i].x + flanks[j].x);
                    float py = 0.5f * (flanks[i].y + flanks[j].y);

                    if ((px * px + py * py) < HORIZON_THRESHOLD) {
>>>>>>> 269b512 (human follower shit)
                        legs_x.push_back(px);
                        legs_y.push_back(py);
                        flanks[i].used = true;
                        flanks[j].used = true;
                    }
                }
            }
        }
    }

<<<<<<< HEAD

    void add_single_legs(const std::vector<Flank>& flanks,
                     std::vector<float>& legs_x,
                     std::vector<float>& legs_y)
    {
        for (const auto& f : flanks)
        {
            if (!f.used)
            {
=======
    void add_single_legs(const std::vector<Flank>& flanks,
                         std::vector<float>& legs_x,
                         std::vector<float>& legs_y)
    {
        for (const auto& f : flanks) {
            if (!f.used && std::isfinite(f.x) && std::isfinite(f.y)) {
>>>>>>> 269b512 (human follower shit)
                legs_x.push_back(f.x);
                legs_y.push_back(f.y);
            }
        }
    }

<<<<<<< HEAD
    void find_leg_hypothesis(
        const sensor_msgs::msg::LaserScan::SharedPtr laser,
        std::vector<float>& legs_x,
        std::vector<float>& legs_y)
=======
    void deduplicate_candidates(std::vector<float>& legs_x, std::vector<float>& legs_y)
    {
        std::vector<float> out_x;
        std::vector<float> out_y;
        const float merge_d2 = 0.10f * 0.10f;

        for (size_t i = 0; i < legs_x.size(); i++) {
            bool merged = false;
            for (size_t j = 0; j < out_x.size(); j++) {
                if (sqdist(legs_x[i], legs_y[i], out_x[j], out_y[j]) < merge_d2) {
                    out_x[j] = 0.5f * (out_x[j] + legs_x[i]);
                    out_y[j] = 0.5f * (out_y[j] + legs_y[i]);
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                out_x.push_back(legs_x[i]);
                out_y.push_back(legs_y[i]);
            }
        }

        legs_x = out_x;
        legs_y = out_y;
    }

    void find_leg_hypothesis(const sensor_msgs::msg::LaserScan::SharedPtr laser,
                             std::vector<float>& legs_x,
                             std::vector<float>& legs_y)
>>>>>>> 269b512 (human follower shit)
    {
        legs_x.clear();
        legs_y.clear();

        auto [laser_x, laser_y] = project_laser_to_base(laser);
<<<<<<< HEAD

        auto flanks = detect_flanks(laser, laser_x, laser_y);

        detect_double_legs(flanks, legs_x, legs_y);

        add_single_legs(flanks, legs_x, legs_y);
    }


    void publish_marker(std::vector<float>& x, std::vector<float>& y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 0.1;
        m.color.r = 1.0; m.color.a = 1.0;
        for(size_t i=0; i<x.size(); i++) {
            geometry_msgs::msg::Point p; p.x = x[i]; p.y = y[i]; p.z = 0.1;
            m.points.push_back(p);
        }
        pub_marker_->publish(m);
=======
        auto flanks = detect_flanks(laser, laser_x, laser_y);

        detect_double_legs(flanks, legs_x, legs_y);
        add_single_legs(flanks, legs_x, legs_y);
        deduplicate_candidates(legs_x, legs_y);
    }

    // =========================
    // Selección de candidatos
    // =========================
    bool get_best_front_candidate(const std::vector<float>& legs_x,
                                  const std::vector<float>& legs_y,
                                  float& best_x, float& best_y)
    {
        float best_score = MAX_FLOAT;
        bool found = false;

        for (size_t i = 0; i < legs_x.size(); i++) {
            float x = legs_x[i];
            float y = legs_y[i];

            if (!inside_front_window(x, y))
                continue;

            float dist = std::sqrt(x * x + y * y);
            float score = dist + 0.5f * std::fabs(y);

            if (score < best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
                found = true;
            }
        }
        return found;
    }

    Candidate get_best_candidate_in_gate(const std::vector<float>& legs_x,
                                         const std::vector<float>& legs_y,
                                         float ref_x, float ref_y,
                                         float gate,
                                         bool apply_lateral_check)
    {
        Candidate best;
        float gate2 = gate * gate;

        for (size_t i = 0; i < legs_x.size(); i++) {
            float x = legs_x[i];
            float y = legs_y[i];

            if (!inside_tracking_window(x, y))
                continue;

            if (apply_lateral_check && std::fabs(y - ref_y) > max_lateral_jump_)
                continue;

            float d2 = sqdist(x, y, ref_x, ref_y);
            if (d2 > gate2)
                continue;

            float dist = std::sqrt(d2);
            float score = 1.0f * dist + 0.18f * std::fabs(x - ref_x) + 0.40f * std::fabs(y - ref_y);

            if (!best.valid || score < best.score) {
                best.valid = true;
                best.x = x;
                best.y = y;
                best.dist_pred = dist;
                best.score = score;
            }
        }

        return best;
    }

    bool accept_measurement(float mx, float my, float pred_x, float pred_y) const
    {
        float jump = std::sqrt(sqdist(mx, my, pred_x, pred_y));
        float speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);

        if (speed < stationary_speed_thr_)
            return jump < stationary_jump_reject_;
        return jump < moving_jump_reject_;
    }

    // =========================
    // Filtro y predicción
    // =========================
    void update_filters(float x, float y)
    {
        x_in_.insert(x_in_.begin(), x);
        y_in_.insert(y_in_.begin(), y);
        x_in_.pop_back();
        y_in_.pop_back();

        x_out_.insert(x_out_.begin(), 0.0f);
        y_out_.insert(y_out_.begin(), 0.0f);
        x_out_.pop_back();
        y_out_.pop_back();
    }

    void apply_butterworth(float& fx, float& fy)
    {
        x_out_[0]  = BFB0X*x_in_[0] + BFB1X*x_in_[1] + BFB2X*x_in_[2] + BFB3X*x_in_[3];
        x_out_[0] -= BFA1X*x_out_[1] + BFA2X*x_out_[2] + BFA3X*x_out_[3];

        y_out_[0]  = BFB0Y*y_in_[0] + BFB1Y*y_in_[1] + BFB2Y*y_in_[2] + BFB3Y*y_in_[3];
        y_out_[0] -= BFA1Y*y_out_[1] + BFA2Y*y_out_[2] + BFA3Y*y_out_[3];

        fx = x_out_[0];
        fy = y_out_[0];
    }

    void publish_track_point(const builtin_interfaces::msg::Time& stamp, float x, float y)
    {
        update_filters(x, y);

        float fx = 0.0f;
        float fy = 0.0f;
        //apply_butterworth(fx, fy);

        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.point.x = fx;
        msg.point.y = fy;
        msg.point.z = 0.3;
        pub_legs_pose_->publish(msg);

        track_x_ = fx;
        track_y_ = fy;

        float alpha_pos = 0.15; // Ajusta entre 0.1 (muy suave) y 0.3 (rápido)
        track_x_ = (alpha_pos * fx) + (1.0f - alpha_pos) * track_x_;
        track_y_ = (alpha_pos * fy) + (1.0f - alpha_pos) * track_y_;
    }

    void update_velocity(float meas_x, float meas_y, double dt)
    {
        if (dt <= 1e-3)
            return;

        float raw_vx = static_cast<float>((meas_x - track_x_) / dt);
        float raw_vy = static_cast<float>((meas_y - track_y_) / dt);

        raw_vx = std::clamp(raw_vx, static_cast<float>(-max_velocity_), static_cast<float>(max_velocity_));
        raw_vy = std::clamp(raw_vy, static_cast<float>(-max_velocity_), static_cast<float>(max_velocity_));

        vel_x_ = static_cast<float>((1.0 - velocity_alpha_) * vel_x_ + velocity_alpha_ * raw_vx);
        vel_y_ = static_cast<float>((1.0 - velocity_alpha_) * vel_y_ + velocity_alpha_ * raw_vy);
    }

    void predict_track(double dt, float& pred_x, float& pred_y)
    {
        dt = std::clamp(dt, 0.0, 0.25);
        pred_x = track_x_ + vel_x_ * static_cast<float>(dt);
        pred_y = track_y_ + vel_y_ * static_cast<float>(dt);
    }

    // =========================
    // Lógica principal
    // =========================
    void handle_acquisition(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                            const std::vector<float>& legs_x,
                            const std::vector<float>& legs_y)
    {
        float cx = 0.0f;
        float cy = 0.0f;
        bool found_candidate = get_best_front_candidate(legs_x, legs_y, cx, cy);

        if (!found_candidate) {
            acquisition_counter_ = 0;
            return;
        }

        if (acquisition_counter_ == 0) {
            track_x_ = cx;
            track_y_ = cy;
            acquisition_counter_ = 1;
            return;
        }

        bool consistent = sqdist(cx, cy, track_x_, track_y_) < (0.18f * 0.18f);

        if (consistent) {
            acquisition_counter_++;
            track_x_ = static_cast<float>((1.0 - measurement_alpha_) * track_x_ + measurement_alpha_ * cx);
            track_y_ = static_cast<float>((1.0 - measurement_alpha_) * track_y_ + measurement_alpha_ * cy);
        } else {
            acquisition_counter_ = 1;
            track_x_ = cx;
            track_y_ = cy;
        }

        if (acquisition_counter_ >= acquisition_frames_) {
            legs_found_ = true;
            lost_counter_ = 0;
            stable_match_count_ = 0;
            vel_x_ = 0.0f;
            vel_y_ = 0.0f;
            track_confidence_ = 0.65f;

            preferred_x_ = track_x_;
            preferred_y_ = track_y_;
            have_preferred_candidate_ = true;

            reset_filters(track_x_, track_y_);
            publish_track_point(msg->header.stamp, track_x_, track_y_);
            last_stamp_ = rclcpp::Time(msg->header.stamp);

            RCLCPP_INFO(this->get_logger(), "Legs acquired at x=%.2f y=%.2f", track_x_, track_y_);
        }
    }

    void handle_tracking(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                         const std::vector<float>& legs_x,
                         const std::vector<float>& legs_y)
    {
        rclcpp::Time now_stamp(msg->header.stamp);
        double dt = (now_stamp - last_stamp_).seconds();
        if (dt <= 0.0 || dt > 0.5)
            dt = 0.033;

        float pred_x = track_x_;
        float pred_y = track_y_;
        predict_track(dt, pred_x, pred_y);

        float strong_gate = adaptive_gate(pred_x, strong_gate_near_, strong_gate_far_);
        float weak_gate   = adaptive_gate(pred_x, weak_gate_near_, weak_gate_far_);

        Candidate strong_best = get_best_candidate_in_gate(legs_x, legs_y, pred_x, pred_y, strong_gate, true);
        Candidate weak_best   = get_best_candidate_in_gate(legs_x, legs_y, pred_x, pred_y, weak_gate, true);

        Candidate preferred_match;
        if (have_preferred_candidate_) {
            preferred_match = get_best_candidate_in_gate(
                legs_x, legs_y, preferred_x_, preferred_y_, weak_gate, false);
        }

        bool accepted = false;
        float chosen_x = pred_x;
        float chosen_y = pred_y;
        bool used_strong = false;

        // 1) Preferir continuidad con el candidato previo si sigue siendo razonable
        if (preferred_match.valid) {
            float pref_dist_to_pred = std::sqrt(sqdist(preferred_match.x, preferred_match.y, pred_x, pred_y));
            bool pref_is_strong = pref_dist_to_pred < strong_gate;

            if (pref_is_strong && accept_measurement(preferred_match.x, preferred_match.y, pred_x, pred_y)) {
                chosen_x = preferred_match.x;
                chosen_y = preferred_match.y;
                accepted = true;
                used_strong = true;
            }
        }

        // 2) Si no, usar mejor strong match respecto a predicción
        if (!accepted && strong_best.valid &&
            accept_measurement(strong_best.x, strong_best.y, pred_x, pred_y)) {
            chosen_x = strong_best.x;
            chosen_y = strong_best.y;
            accepted = true;
            used_strong = true;
        }

        // 3) Si solo hay weak match, úsalo muy despacio y sin cambiar ID fácil
        if (!accepted && weak_best.valid) {
            chosen_x = weak_best.x;
            chosen_y = weak_best.y;
            accepted = true;
            used_strong = false;
        }

        if (accepted) {
            lost_counter_ = 0;

            // Histéresis: no cambies el preferred a menos que sea claramente mejor varias veces
            if (!have_preferred_candidate_) {
                preferred_x_ = chosen_x;
                preferred_y_ = chosen_y;
                have_preferred_candidate_ = true;
                stable_match_count_ = 0;
            } else {
                float old_pref_score = std::sqrt(sqdist(preferred_x_, preferred_y_, pred_x, pred_y));
                float new_score      = std::sqrt(sqdist(chosen_x, chosen_y, pred_x, pred_y));

                bool same_as_pref = std::sqrt(sqdist(chosen_x, chosen_y, preferred_x_, preferred_y_)) < 0.10f;

                if (same_as_pref) {
                    preferred_x_ = static_cast<float>(0.7 * preferred_x_ + 0.3 * chosen_x);
                    preferred_y_ = static_cast<float>(0.7 * preferred_y_ + 0.3 * chosen_y);
                    stable_match_count_ = 0;
                } else {
                    if ((old_pref_score - new_score) > switch_margin_) {
                        stable_match_count_++;
                        if (stable_match_count_ >= switch_confirm_frames_) {
                            preferred_x_ = chosen_x;
                            preferred_y_ = chosen_y;
                            stable_match_count_ = 0;
                        }
                    } else {
                        stable_match_count_ = 0;
                    }
                }
            }

            float meas_x;
            float meas_y;

            if (used_strong) {
                meas_x = static_cast<float>((1.0 - measurement_alpha_) * pred_x + measurement_alpha_ * chosen_x);
                meas_y = static_cast<float>((1.0 - measurement_alpha_) * pred_y + measurement_alpha_ * chosen_y);
                track_confidence_ = std::min(1.0f, track_confidence_ + static_cast<float>(confidence_gain_strong_));
            } else {
                // weak match: actualizar suave
                meas_x = static_cast<float>(0.88 * pred_x + 0.12 * chosen_x);
                meas_y = static_cast<float>(0.88 * pred_y + 0.12 * chosen_y);
                track_confidence_ = std::max(0.0f, track_confidence_ - static_cast<float>(confidence_decay_weak_));
                track_confidence_ = std::min(1.0f, track_confidence_ + static_cast<float>(confidence_gain_weak_));
            }

            update_velocity(meas_x, meas_y, dt);
            publish_track_point(msg->header.stamp, meas_x, meas_y);
            last_stamp_ = now_stamp;
            return;
        }

        // Nada aceptado: predicción pura con degradación gradual
        lost_counter_++;
        track_confidence_ = std::max(0.0f, track_confidence_ - static_cast<float>(confidence_decay_lost_));

        if (lost_counter_ <= max_lost_frames_ && track_confidence_ > min_conf_to_keep_track_) {
            publish_track_point(msg->header.stamp, pred_x, pred_y);
            last_stamp_ = now_stamp;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Leg track lost");
        legs_found_ = false;
        acquisition_counter_ = 0;
        lost_counter_ = 0;
        stable_match_count_ = 0;
        vel_x_ = 0.0f;
        vel_y_ = 0.0f;
        track_confidence_ = 0.0f;
        have_preferred_candidate_ = false;
    }

    // =========================
    // Marcadores
    // =========================
    visualization_msgs::msg::Marker get_hypothesis_marker(const std::vector<float>& legs_x,
                                                          const std::vector<float>& legs_y)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = frame_id_;
        marker.ns = "leg_finder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.a = 1.0;
        marker.color.g = 0.5;
        marker.color.r = 0.1;
        marker.color.b = 0.1;
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);

        for (size_t i = 0; i < legs_x.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = legs_x[i];
            p.y = legs_y[i];
            p.z = 0.3;
            marker.points.push_back(p);
        }

        return marker;
>>>>>>> 269b512 (human follower shit)
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegFinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> 269b512 (human follower shit)
