#include <iostream>
#include <vector>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Geometry>

// ====== CONSTANTES ======
#define FILTER_THRESHOLD  .081
#define FLANK_THRESHOLD  .04
#define HORIZON_THRESHOLD  9
#define MAX_FLOAT  57295779500
#define LEG_THIN  0.00341
#define LEG_THICK  0.03
#define TWO_LEGS_THIN  0.056644
#define TWO_LEGS_THICK  0.25
#define TWO_LEGS_NEAR  0.0022201
#define TWO_LEGS_FAR  0.25
#define IS_LEG_THRESHOLD 0.5
#define IN_FRONT_MIN_X  0.25
#define IN_FRONT_MAX_X  2.5
#define IN_FRONT_MIN_Y -0.5
#define IN_FRONT_MAX_Y  0.5

//BUTTERWORTH FILTER A Ó B EN X O Y
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

struct Flank{
    float x;
    float y;
    bool used = false;
};

class LegFinder : public rclcpp::Node
{
public:
    LegFinder() : Node("leg_finder"), legs_found_(false), stop_robot_(false),
                  legs_in_front_cnt_(0), legs_lost_counter_(0), set_stage_(true)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING LEG FINDER");

        laser_scan_topic_ = this->declare_parameter("laser_scan_topic", "/scan");
        frame_id_         = this->declare_parameter("frame_id", "base_link");
        show_hypothesis_  = this->declare_parameter("show_hypothesis", true);
        scan_downsampling_ = this->declare_parameter("scan_downsampling", 1);
        //TODO: to auto set the laser scan frame from the first msg topic
        //laser_scan_frame_ = this->declare_parameter("laser_scan_frame", "laser_link");

        pub_legs_hypothesis_ = this->create_publisher<visualization_msgs::msg::Marker>("/hri/leg_finder/hypothesis", 1);
        pub_legs_pose_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hri/leg_finder/leg_pose", 1);
        pub_legs_found_ = this->create_publisher<std_msgs::msg::Bool>("/hri/leg_finder/legs_found", 1);

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1,
            std::bind(&LegFinder::callback_enable, this, std::placeholders::_1));

        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1,
            std::bind(&LegFinder::callback_stop, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        legs_x_filter_input_.assign(4, 0.0);
        legs_x_filter_output_.assign(4, 0.0);
        legs_y_filter_input_.assign(4, 0.0);
        legs_y_filter_output_.assign(4, 0.0);


        RCLCPP_INFO(this->get_logger(), "Ready. show_hypothesis=%s, laser_topic=%s",
                    show_hypothesis_ ? "True" : "False", laser_scan_topic_.c_str());
    }

private:
    // === Variables ROS ===
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_legs_hypothesis_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_legs_found_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool set_stage_;
    bool legs_found_;
    bool stop_robot_;
    int legs_in_front_cnt_;
    int legs_lost_counter_;
    float last_legs_pose_x_, last_legs_pose_y_;
    std::vector<float> legs_x_filter_input_, legs_x_filter_output_;
    std::vector<float> legs_y_filter_input_, legs_y_filter_output_;

    std::string frame_id_;
    std::string laser_scan_topic_;
    std::string laser_scan_frame_;
    bool show_hypothesis_;
    int scan_downsampling_;

    // ===== CALLBACKS =====
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                laser_scan_topic_, 1, std::bind(&LegFinder::callback_scan, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Enabled leg finder");
        }
        else
        {
            sub_laser_.reset();
            legs_found_ = false;
            legs_in_front_cnt_ = 0;
            RCLCPP_INFO(this->get_logger(), "Disabled leg finder");
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr)
    {
        sub_laser_.reset();
        legs_found_ = false;
        legs_in_front_cnt_ = 0;
        RCLCPP_INFO(this->get_logger(), "Stopped leg finder");
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Auto-set laser scan frame from the first message
        if (set_stage_){
            laser_scan_frame_ = msg->header.frame_id;
            set_stage_ = false;
            RCLCPP_INFO(this->get_logger(), "Laser scan frame set to %s", laser_scan_frame_.c_str());
        }

        // Downsample and filter laser scan
        std::vector<float> ranges = msg->ranges;
        if (scan_downsampling_ > 1)
            ranges = downsample_scan(ranges, scan_downsampling_);
        ranges = filter_laser_ranges(ranges);

        // Get leg hypotheses
        std::vector<float> legs_x, legs_y;
        find_leg_hypothesis(msg, legs_x, legs_y);

        // publish leg hypotheses markers
        if (show_hypothesis_){
            auto marker = get_hypothesis_marker(legs_x, legs_y);
            pub_legs_hypothesis_->publish(marker);
        }

        // Get nearest leg in front of the robot
        float nearest_x = MAX_FLOAT, nearest_y = MAX_FLOAT;
        bool leg_in_front = get_nearest_legs_in_front(legs_x, legs_y, nearest_x, nearest_y);

        // Logic to find and track legs
        if (!legs_found_ && !stop_robot_)
        {
            handle_leg_acquisition(leg_in_front, nearest_x, nearest_y);
        }
        else if (legs_found_)
        {
            handle_leg_tracking(legs_x, legs_y, nearest_x, nearest_y);
        }
        else
            stop_robot_ = true;
    // Publish legs state
    std_msgs::msg::Bool msg_found;
    msg_found.data = legs_found_;
    pub_legs_found_->publish(msg_found);

    }

    // ===== FUNCIONES AUXILIARES =====

    std::vector<float> downsample_scan(std::vector<float>& ranges, int downsampling)
    {
        std::vector<float> new_scans(ranges.size() / downsampling);
        for (size_t i = 0; i < ranges.size(); i += downsampling)
            new_scans[i / downsampling] = ranges[i];
        return new_scans;
    }

    std::vector<float> filter_laser_ranges(std::vector<float>& laser_ranges)
    {
        std::vector<float> filtered(laser_ranges.size(), 0.0);
        for (size_t i = 1; i + 1 < laser_ranges.size(); ++i)
        {
            if (laser_ranges[i] < 0.4)
                continue;
            if (fabs(laser_ranges[i - 1] - laser_ranges[i]) < FILTER_THRESHOLD &&
                fabs(laser_ranges[i] - laser_ranges[i + 1]) < FILTER_THRESHOLD)
                filtered[i] = (laser_ranges[i - 1] + laser_ranges[i] + laser_ranges[i + 1]) / 3.0;
            else
                filtered[i] = 0;
        }
        return filtered;
    }

    bool is_leg(float x1, float y1, float x2, float y2)
    {
        bool result = false;
        float m1, m2, px, py, angle;
        if(x1 != x2) m1 = (y1 - y2)/(x1 - x2);
        else m1 = MAX_FLOAT;

        px = (x1 + x2) / 2;
        py = (y1 + y2) / 2;
        if((px*px + py*py) < HORIZON_THRESHOLD)
        {
            if(px != 0)
                m2 = py / px;
            else
                m2 = MAX_FLOAT;
            angle = fabs((m2 - m1) / (1 + (m2*m1)));
            if(angle > IS_LEG_THRESHOLD)
                result = true;
        }
        return result;
    }

    bool obst_in_front(sensor_msgs::msg::LaserScan::SharedPtr laser, float xmin, float xmax, float ymin, float ymax, float thr){
        float theta = laser->angle_min;
        float quantize = 0.0;
        for(size_t i=0; i < laser->ranges.size(); i++)
        {
            float x, y;
            theta = laser->angle_min + i*laser->angle_increment;
            x = laser->ranges[i] * cos(theta);
            y = laser->ranges[i] * sin(theta);
            if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
                quantize += laser->ranges[i];
        }
        //std::cout << "leg_finder_node.-> quantize : " << quantize << std::endl;
        if(quantize >= thr)
            return true;
    }
    
    Eigen::Affine3d get_lidar_position()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf = tf_buffer_->lookupTransform("base_link", laser_scan_frame_, tf2::TimePointZero);
        return tf2::transformToEigen(tf.transform);
    }

    visualization_msgs::msg::Marker get_hypothesis_marker(std::vector<float>& legs_x, std::vector<float>& legs_y)
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
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        marker.points.resize(legs_x.size());
        for (size_t i = 0; i < legs_x.size(); ++i)
        {
            marker.points[i].x = legs_x[i];
            marker.points[i].y = legs_y[i];
            marker.points[i].z = 0.3;
        }
        return marker;
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
        int ant2 = 0;

        for (int i = 1; i < (int)laser->ranges.size(); i++)
        {
            int ant = ant2;

            if (fabs(laser->ranges[i] - laser->ranges[i-1]) > FLANK_THRESHOLD)
                ant2 = i;

            if (fabs(laser->ranges[i] - laser->ranges[i-1]) > FLANK_THRESHOLD &&
                (is_leg(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) ||
                is_leg(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2])))
            {
                float dx = laser_x[ant] - laser_x[i-1];
                float dy = laser_y[ant] - laser_y[i-1];
                float dist2 = dx*dx + dy*dy;

                float sum_x = 0, sum_y = 0;
                for (int j = ant; j < i; j++)
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
        for (int i = 0; i < (int)flanks.size() - 1; i++)
        {
            for (int j = i + 1; j < (int)flanks.size(); j++)
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

    bool get_nearest_legs_in_front(std::vector<float>& legs_x, std::vector<float>& legs_y, float& nearest_x, float& nearest_y)
    {
        nearest_x = MAX_FLOAT;
        nearest_y = MAX_FLOAT;
        float min_dist = MAX_FLOAT;
        for(int i=0; i < legs_x.size(); i++)
        {
            if(!(legs_x[i] > IN_FRONT_MIN_X && legs_x[i] < IN_FRONT_MAX_X && legs_y[i] > IN_FRONT_MIN_Y && legs_y[i] < IN_FRONT_MAX_Y))
                continue;
            float dist = sqrt(legs_x[i]*legs_x[i] + legs_y[i]*legs_y[i]);
            if(dist < min_dist)
            {
                min_dist = dist;
                nearest_x = legs_x[i];
                nearest_y = legs_y[i];
            }
        }
        return nearest_x > IN_FRONT_MIN_X && nearest_x < IN_FRONT_MAX_X && nearest_y > IN_FRONT_MIN_Y && nearest_y < IN_FRONT_MAX_Y;
    }

    bool get_nearest_legs_to_last_legs(std::vector<float>& legs_x, std::vector<float>& legs_y, float& nearest_x,
            float& nearest_y, float last_x, float last_y)
    {
        nearest_x = MAX_FLOAT;
        nearest_y = MAX_FLOAT;
        float min_dist = MAX_FLOAT;
        for(int i=0; i < legs_x.size(); i++)
        {
            float dist = sqrt((legs_x[i] - last_x)*(legs_x[i] - last_x) + (legs_y[i] - last_y)*(legs_y[i] - last_y));
            if(dist < min_dist)
            {
                min_dist = dist;
                nearest_x = legs_x[i];
                nearest_y = legs_y[i];
            }
        }
        return min_dist < 0.33;
    }

    void handle_leg_acquisition(bool legs_in_front, float nearest_x, float nearest_y)
    {
        if (legs_in_front) legs_in_front_cnt_++;
        if (legs_in_front_cnt_ > 20) {
            legs_found_ = true;
            legs_lost_counter_ = 0;
            last_legs_pose_x_ = nearest_x;
            last_legs_pose_y_ = nearest_y;
            // Inicializar filtros
            for (int i 
                = 0; i < 4; i++) {
                legs_x_filter_input_[i] = nearest_x;
                legs_x_filter_output_[i] = nearest_x;
                legs_y_filter_input_[i] = nearest_y;
                legs_y_filter_output_[i] = nearest_y;
            }
        }
    }

    void handle_leg_tracking(std::vector<float>& legs_x, std::vector<float>& legs_y,
                            float nearest_x, float nearest_y)
    {
        geometry_msgs::msg::PointStamped filtered_legs;
        filtered_legs.header.frame_id = frame_id_;
        filtered_legs.point.z = 0.3;

        bool publish_legs = false;
        bool fobst_in_front = false; // Puedes usar obst_in_front() si quieres detección de obstáculos

        if (!fobst_in_front) {
            if (get_nearest_legs_to_last_legs(legs_x, legs_y, nearest_x, nearest_y, last_legs_pose_x_, last_legs_pose_y_)) {
                update_filters(nearest_x, nearest_y);
                publish_legs = true;
            } else {
                update_filters(last_legs_pose_x_, last_legs_pose_y_);
                if (++legs_lost_counter_ > 20) {
                    legs_found_ = false;
                    legs_in_front_cnt_ = 0;
                }
            }

            // Aplicar Butterworth
            apply_butterworth(filtered_legs);

            if (publish_legs) pub_legs_pose_->publish(filtered_legs);
        } else {
            stop_robot_ = true;
        }
    }

    void update_filters(float x, float y)
    {
        legs_x_filter_input_.insert(legs_x_filter_input_.begin(), x);
        legs_y_filter_input_.insert(legs_y_filter_input_.begin(), y);
        legs_x_filter_input_.pop_back();
        legs_y_filter_input_.pop_back();
        legs_x_filter_output_.pop_back();
        legs_y_filter_output_.pop_back();
        legs_x_filter_output_.insert(legs_x_filter_output_.begin(), 0);
        legs_y_filter_output_.insert(legs_y_filter_output_.begin(), 0);

        last_legs_pose_x_ = x;
        last_legs_pose_y_ = y;
    }

    void apply_butterworth(geometry_msgs::msg::PointStamped& filtered_legs)
    {
        legs_x_filter_output_[0]  = BFB0X*legs_x_filter_input_[0] + BFB1X*legs_x_filter_input_[1] +
                                    BFB2X*legs_x_filter_input_[2] + BFB3X*legs_x_filter_input_[3];
        legs_x_filter_output_[0] -= BFA1X*legs_x_filter_output_[1] + BFA2X*legs_x_filter_output_[2] + BFA3X*legs_x_filter_output_[3];

        legs_y_filter_output_[0]  = BFB0Y*legs_y_filter_input_[0] + BFB1Y*legs_y_filter_input_[1] +
                                    BFB2Y*legs_y_filter_input_[2] + BFB3Y*legs_y_filter_input_[3];
        legs_y_filter_output_[0] -= BFA1Y*legs_y_filter_output_[1] + BFA2Y*legs_y_filter_output_[2] + BFA3Y*legs_y_filter_output_[3];

        filtered_legs.point.x = legs_x_filter_output_[0];
        filtered_legs.point.y = legs_y_filter_output_[0];
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
