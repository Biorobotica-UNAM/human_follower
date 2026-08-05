// human_tracking_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class HumanTracker : public rclcpp::Node
{
public:
    HumanTracker()
    : Node("human_tracking_node"), enable_(false), last_confidence_(0.0f)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING HUMAN TRACKING NODE...");

        watchdog_timeout_s_ = this->declare_parameter("watchdog_timeout_s", 15.0); 
        min_confidence_warn_= this->declare_parameter("min_confidence_warn", 0.2);
        enable_watchdog_    = this->declare_parameter("enable_watchdog", true);

        pub_hf_enable_  = this->create_publisher<std_msgs::msg::Bool>("/hri/human_following/enable", 1);
        pub_lf_enable_  = this->create_publisher<std_msgs::msg::Bool>("/hri/leg_finder/enable", 1);
        pub_vision_enable_ = this->create_publisher<std_msgs::msg::Bool>("/vision/enable_continuous", 1);
        pub_tracking_state_ = this->create_publisher<std_msgs::msg::Bool>("/hri/human_tracking/state", 10);

        // cliente de focus 
        cli_focus_enable_ = this->create_client<std_srvs::srv::Trigger>("/vision/focus/enable");
        cli_focus_disable_ = this->create_client<std_srvs::srv::Trigger>("/vision/focus/disable");

        sub_confidence_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hri/human_tracking/confidence", 10,
            std::bind(&HumanTracker::callback_confidence, this, std::placeholders::_1));

        human_tracking_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/hri/human_tracking/enable",
            std::bind(&HumanTracker::callback_enable_tracking, this,
                      std::placeholders::_1, std::placeholders::_2));

        watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&HumanTracker::watchdog_tick, this));

        last_confidence_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(),
            "Human tracking node ready. Watchdog: %s (timeout=%.1fs)",
            enable_watchdog_ ? "ON" : "OFF", watchdog_timeout_s_);
    }

private:
    double watchdog_timeout_s_;
    double min_confidence_warn_;
    bool   enable_watchdog_;

    bool   enable_;
    bool   follower_enabled_ = false; 
    float  last_confidence_;
    rclcpp::Time last_confidence_time_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_hf_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_lf_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_vision_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_tracking_state_;
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr   cli_focus_enable_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr   cli_focus_disable_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_confidence_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   human_tracking_srv_;
    rclcpp::TimerBase::SharedPtr                          watchdog_timer_;

    void publish_enable_to_all(bool value)
    {
        std_msgs::msg::Bool msg;
        msg.data = value;
        pub_hf_enable_->publish(msg);
        pub_lf_enable_->publish(msg);
        pub_vision_enable_->publish(msg);
        pub_tracking_state_->publish(msg);

        if (value) {
            trigger_vision_service(cli_focus_enable_, "Enabling vision focus...");
        } else {
            trigger_vision_service(cli_focus_disable_, "Disabling vision focus...");
        }
    }
   // callback para focus de visión, se llama desde el servicio de enable/disable tracking
    void trigger_vision_service(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const std::string & action_name)
    {
        if (client->wait_for_service(std::chrono::milliseconds(300))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            
            client->async_send_request(req, [this, action_name](std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> future) {
                auto res = future.get();
                if (res->success) {
                    RCLCPP_INFO(this->get_logger(), "Vision focus %s: %s", action_name.c_str(), res->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Vision focus %s failed: %s", action_name.c_str(), res->message.c_str());
                }
            });
        } else {
            RCLCPP_WARN(this->get_logger(), "Vision focus service for %s not available!", action_name.c_str());
        }
    }

    void callback_enable_tracking(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        enable_ = request->data;
        publish_enable_to_all(enable_);

        if (enable_) {
            last_confidence_time_ = this->get_clock()->now();
            follower_enabled_ = true;  
        } else {
            follower_enabled_ = false; 
        }

        response->success = true;
        response->message = enable_ ? "Human tracking ENABLED" : "Human tracking DISABLED";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void callback_confidence(const std_msgs::msg::Float32::SharedPtr msg)
    {
        last_confidence_ = msg->data;

        if (last_confidence_ > min_confidence_warn_) {
            last_confidence_time_ = this->get_clock()->now();
        }

        if (enable_ && last_confidence_ < min_confidence_warn_ && last_confidence_ > 0.0f) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Baja confianza en tracking: %.2f — target incierto", last_confidence_);
        }
    }

    void watchdog_tick()
    {
        if (!enable_ || !enable_watchdog_) return;

        double elapsed = (this->get_clock()->now() - last_confidence_time_).seconds();

        if (elapsed > watchdog_timeout_s_ && follower_enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "WATCHDOG: Sin detección por %.1f s — pausando follower (sensores activos)", elapsed);
            follower_enabled_ = false;
            std_msgs::msg::Bool stop;
            stop.data = false;
            pub_hf_enable_->publish(stop);
            last_confidence_time_ = this->get_clock()->now();
        }
        else if (elapsed < (watchdog_timeout_s_ * 0.5) && !follower_enabled_ && enable_) {
            RCLCPP_INFO(this->get_logger(),
                "WATCHDOG: Datos recuperados — re-activando human follower");
            follower_enabled_ = true;
            last_confidence_time_ = this->get_clock()->now();
            std_msgs::msg::Bool go;
            go.data = true;
            pub_hf_enable_->publish(go);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanTracker>());
    rclcpp::shutdown();
    return 0;
}
