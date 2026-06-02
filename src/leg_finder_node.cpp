// leg_finder_node.cpp — versión simplificada
// Quitado: Butterworth, coasting, gate adaptiva por velocidad, diagnóstico de clusters
// Conservado: detección de flancos, clusters, selección de par de piernas, Kalman no (era del follower)

#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

struct Flank   { int index; std::string type; };
struct Cluster {
    geometry_msgs::msg::PointStamped center;
    double width = 0.0;
    double probability = 0.0;
};

class LegFinderNode : public rclcpp::Node
{
public:
    LegFinderNode() : Node("leg_finder_node"), enable_(false)
    {
        this->declare_parameter<std::string>("scan_topic",       "/scan");
        this->declare_parameter<std::string>("base_frame",       "base_link");
        this->declare_parameter<std::string>("legs_pose_topic",  "/hri/leg_finder/legs_pose");

        scan_topic_      = this->get_parameter("scan_topic").as_string();
        base_frame_      = this->get_parameter("base_frame").as_string();
        legs_pose_topic_ = this->get_parameter("legs_pose_topic").as_string();

        flank_threshold_   = this->declare_parameter("flank_threshold",   0.04);
        horizon_threshold_ = this->declare_parameter("horizon_threshold", 9);
        leg_thin_          = this->declare_parameter("leg_thin",          0.00341);
        leg_thick_         = this->declare_parameter("leg_thick",         0.03);
        two_legs_thin_     = this->declare_parameter("two_legs_thin",     0.056644);
        two_legs_thick_    = this->declare_parameter("two_legs_thick",    0.25);
        two_legs_near_     = this->declare_parameter("two_legs_near",     0.0022201);
        two_legs_far_      = this->declare_parameter("two_legs_far",      0.25);
        in_front_min_x_    = this->declare_parameter("in_front_min_x",   0.25);
        in_front_max_x_    = this->declare_parameter("in_front_max_x",   2.5);
        in_front_min_y_    = this->declare_parameter("in_front_min_y",   -0.5);
        in_front_max_y_    = this->declare_parameter("in_front_max_y",   0.5);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_scan_   = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LegFinderNode::scan_callback, this, std::placeholders::_1));
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1,
            std::bind(&LegFinderNode::enable_callback, this, std::placeholders::_1));

        pub_legs_  = this->create_publisher<geometry_msgs::msg::PointStamped>(legs_pose_topic_, 10);
        pub_marker_= this->create_publisher<visualization_msgs::msg::Marker>("/hri/leg_finder/marker", 10);

        RCLCPP_INFO(this->get_logger(), "LegFinder ready.");
    }

private:
    std::string scan_topic_, base_frame_, legs_pose_topic_;
    bool enable_;

    double flank_threshold_, leg_thin_, leg_thick_;
    double two_legs_thin_, two_legs_thick_, two_legs_near_, two_legs_far_;
    double in_front_min_x_, in_front_max_x_, in_front_min_y_, in_front_max_y_;
    int    horizon_threshold_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  sub_scan_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           sub_enable_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_marker_;

    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg) { enable_ = msg->data; }

    Eigen::Affine3d get_lidar_transform()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(base_frame_, "laser_frame",
                         rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(100)));
        } catch (...) {
            tf_msg = tf_buffer_->lookupTransform(base_frame_, "cloud_lidar_link",
                         rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(100)));
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
            geometry_msgs::msg::PointStamped pt;
            if (rho < scan->range_min || rho > scan->range_max) {
                pt.point.x = pt.point.y = pt.point.z = std::numeric_limits<double>::quiet_NaN();
            } else {
                double theta = scan->angle_min + i * scan->angle_increment;
                Eigen::Vector3d p = T * Eigen::Vector3d(rho*std::cos(theta), rho*std::sin(theta), 0.0);
                pt.header.frame_id = base_frame_;
                pt.header.stamp    = scan->header.stamp;
                pt.point.x = p.x(); pt.point.y = p.y(); pt.point.z = p.z();
            }
            out.push_back(pt);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!enable_) return;

        Eigen::Affine3d T;
        try { T = get_lidar_transform(); } catch (...) { return; }

        std::vector<geometry_msgs::msg::PointStamped> pts;
        project_laser_to_base(scan, pts, T);

        // Flancos
        std::vector<Flank> flanks;
        for (size_t i = 1; i + 1 < scan->ranges.size(); ++i) {
            double rp = scan->ranges[i-1], rc = scan->ranges[i], rn = scan->ranges[i+1];
            if (rc < scan->range_min || rc > scan->range_max) continue;
            if (std::abs(rc-rp) > flank_threshold_ || std::abs(rn-rc) > flank_threshold_)
                flanks.push_back({static_cast<int>(i),
                    ((rc-rp)<0||(rn-rc)<0) ? "CONCAVE" : "CONVEX"});
        }

        // Clusters
        std::vector<Cluster> clusters;
        for (size_t i = 0; i < flanks.size(); ++i) {
            if (flanks[i].type != "CONCAVE") continue;
            for (size_t j = i+1; j < flanks.size(); ++j) {
                if (flanks[j].type != "CONVEX") continue;
                int s = flanks[i].index, e = flanks[j].index;
                if (e - s > horizon_threshold_) continue;
                double sx=0, sy=0, fx=0, fy=0, lx=0, ly=0; int n=0; bool gf=false;
                for (int k=s; k<=e; ++k) {
                    if (k<0||k>=(int)pts.size()||std::isnan(pts[k].point.x)) continue;
                    sx+=pts[k].point.x; sy+=pts[k].point.y;
                    if (!gf){fx=pts[k].point.x;fy=pts[k].point.y;gf=true;}
                    lx=pts[k].point.x; ly=pts[k].point.y; n++;
                }
                if (!n) continue;
                Cluster c;
                c.center.header.stamp    = scan->header.stamp;
                c.center.header.frame_id = base_frame_;
                c.center.point.x=sx/n; c.center.point.y=sy/n; c.center.point.z=0;
                c.width=std::hypot(lx-fx, ly-fy);
                clusters.push_back(c); break;
            }
        }

        // Clasificar como pierna
        std::vector<Cluster> legs;
        for (auto& c : clusters) {
            if      (c.width >= leg_thin_  && c.width <= leg_thick_)      c.probability = 0.4;
            else if (c.width >  leg_thick_ && c.width <= two_legs_thick_) c.probability = 0.2;
            if (c.probability > 0.1 &&
                c.center.point.x >= in_front_min_x_ && c.center.point.x <= in_front_max_x_ &&
                c.center.point.y >= in_front_min_y_ && c.center.point.y <= in_front_max_y_)
                legs.push_back(c);
        }

        geometry_msgs::msg::PointStamped pose;
        bool found = false;

        // Buscar par
        if (legs.size() >= 2) {
            double best = -1.0;
            for (size_t i=0; i<legs.size(); ++i) for (size_t j=i+1; j<legs.size(); ++j) {
                double d = std::hypot(legs[i].center.point.x-legs[j].center.point.x,
                                      legs[i].center.point.y-legs[j].center.point.y);
                if (d < two_legs_near_ || d > two_legs_far_) continue;
                double wd = std::abs(legs[i].width - legs[j].width);
                double score = (legs[i].probability+legs[j].probability) / (1.0+wd*10.0);
                if (score > best) {
                    best = score;
                    pose.header    = legs[i].center.header;
                    pose.point.x   = (legs[i].center.point.x+legs[j].center.point.x)/2.0;
                    pose.point.y   = (legs[i].center.point.y+legs[j].center.point.y)/2.0;
                    pose.point.z   = 0.0;
                    found = true;
                }
            }
        }

        // Fallback: mejor pierna sola
        if (!found && !legs.empty()) {
            double best = -1.0;
            for (const auto& l : legs)
                if (l.probability > best) { best=l.probability; pose=l.center; found=true; }
        }

        if (found) {
            pub_legs_->publish(pose);
            visualization_msgs::msg::Marker m;
            m.header.frame_id=base_frame_; m.header.stamp=this->get_clock()->now();
            m.ns="legs"; m.id=0; m.type=visualization_msgs::msg::Marker::CYLINDER;
            m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=pose.point.x; m.pose.position.y=pose.point.y; m.pose.position.z=0.2;
            m.scale.x=m.scale.y=0.2; m.scale.z=0.4;
            m.color.a=1.0f; m.color.g=1.0f;
            pub_marker_->publish(m);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegFinderNode>());
    rclcpp::shutdown();
    return 0;
}