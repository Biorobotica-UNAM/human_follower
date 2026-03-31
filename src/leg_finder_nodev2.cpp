#include <iostream>
#include <vector>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

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
    float x;
    float y;
    bool used = false;
};

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

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    }

    Eigen::Affine3d get_lidar_position()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf = tf_buffer_->lookupTransform("base_link", laser_scan_frame_, tf2::TimePointZero);
        return tf2::transformToEigen(tf.transform);
    }

    std::pair<std::vector<float>, std::vector<float>>
    project_laser_to_base(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
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
        }

        return {laser_x, laser_y};
    }

    std::vector<Flank> detect_flanks(
        const sensor_msgs::msg::LaserScan::SharedPtr laser,
        const std::vector<float>& laser_x,
        const std::vector<float>& laser_y)
    {
        std::vector<Flank> flanks;
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
        }

        return flanks;
    }

    void detect_double_legs(std::vector<Flank>& flanks,
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
                        legs_x.push_back(px);
                        legs_y.push_back(py);
                        flanks[i].used = true;
                        flanks[j].used = true;
                    }
                }
            }
        }
    }


    void add_single_legs(const std::vector<Flank>& flanks,
                     std::vector<float>& legs_x,
                     std::vector<float>& legs_y)
    {
        for (const auto& f : flanks)
        {
            if (!f.used)
            {
                legs_x.push_back(f.x);
                legs_y.push_back(f.y);
            }
        }
    }

    void find_leg_hypothesis(
        const sensor_msgs::msg::LaserScan::SharedPtr laser,
        std::vector<float>& legs_x,
        std::vector<float>& legs_y)
    {
        legs_x.clear();
        legs_y.clear();

        auto [laser_x, laser_y] = project_laser_to_base(laser);

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
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegFinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
