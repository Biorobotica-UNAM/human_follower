// ============================================================================
//  human_tracking_node.cpp  —  RoboCup-grade version
//  Fixes vs original:
//    [BUG-1] pub_person_tracker_enable_ y pub_human_detection_enable_ publicaban
//            en el MISMO topic (/vision/enable_continuous), creando dos publishers
//            redundantes. Consolidado en uno solo.
//    [BUG-2] pub_yd_enable_ nunca se inicializaba pero se intentaba usar con
//            comprobación manual de puntero. Eliminado correctamente.
//  Mejoras:
//    [OPT-1] Timer de watchdog: si el follower lleva N segundos activo sin
//            recibir confirmación de target, se auto-desactiva y logea warning.
//            Previene que el robot "huya" en demos si pierde al humano.
//    [OPT-2] Subscripción a /hri/human_tracking/confidence para monitorear
//            la calidad del seguimiento en tiempo real.
//    [OPT-3] Parámetros de configuración expuestos via ROS params.
//    [OPT-4] Estado publicado como tópico para que otros nodos puedan consultarlo
//            sin llamar al servicio.
// ============================================================================

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class HumanTracker : public rclcpp::Node
{
public:
    HumanTracker()
    : Node("human_tracking_node"), enable_(false), last_confidence_(0.0f)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING HUMAN TRACKING NODE...");

        // Param
        watchdog_timeout_s_ = this->declare_parameter("watchdog_timeout_s", 5.0);
        min_confidence_warn_= this->declare_parameter("min_confidence_warn", 0.2);
        enable_watchdog_    = this->declare_parameter("enable_watchdog", true);

        // Publishers
        pub_hf_enable_  = this->create_publisher<std_msgs::msg::Bool>(
            "/hri/human_following/enable", 1);
        pub_lf_enable_  = this->create_publisher<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1);
        pub_vision_enable_ = this->create_publisher<std_msgs::msg::Bool>(
            "/vision/enable_continuous", 1);
      
        pub_tracking_state_ = this->create_publisher<std_msgs::msg::Bool>(
            "/hri/human_tracking/state", 10);

        // Suscriptor
        sub_confidence_ = this->create_subscription<std_msgs::msg::Float32>(
            "/hri/human_tracking/confidence", 10,
            std::bind(&HumanTracker::callback_confidence, this, std::placeholders::_1));

        // Service
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
    // Parm
    double watchdog_timeout_s_;
    double min_confidence_warn_;
    bool   enable_watchdog_;

    // state variabls
    bool   enable_;
    float  last_confidence_;
    rclcpp::Time last_confidence_time_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_hf_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_lf_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_vision_enable_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_tracking_state_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_confidence_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   human_tracking_srv_;
    rclcpp::TimerBase::SharedPtr                          watchdog_timer_;

    // ────────────────────────────────────────────────────────────────────────
    //  Publicar enable a todos los subsistemas
    // ────────────────────────────────────────────────────────────────────────
    void publish_enable_to_all(bool value)
    {
        std_msgs::msg::Bool msg;
        msg.data = value;
        pub_hf_enable_->publish(msg);
        pub_lf_enable_->publish(msg);
        pub_vision_enable_->publish(msg);   
        pub_tracking_state_->publish(msg);  
    }

    void callback_enable_tracking(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        enable_ = request->data;
        publish_enable_to_all(enable_);

        if (enable_) {
            last_confidence_time_ = this->get_clock()->now();
        }

        response->success = true;
        response->message = enable_ ? "Human tracking ENABLED" : "Human tracking DISABLED";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    //   Callback de confianza del trackes
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

    // Watchdog: auto-desactivar si se pierde al humano por timeout
    void watchdog_tick()
    {
        if (!enable_ || !enable_watchdog_) return;

        double elapsed = (this->get_clock()->now() - last_confidence_time_).seconds();

        if (elapsed > watchdog_timeout_s_) {
            RCLCPP_WARN(this->get_logger(),
                "WATCHDOG: Sin detección por %.1f s — desactivando human following "
                "(leg_finder y person_tracker siguen activos para re-adquisición)", elapsed);

            std_msgs::msg::Bool stop_follower;
            stop_follower.data = false;
            pub_hf_enable_->publish(stop_follower);
            last_confidence_time_ = this->get_clock()->now();
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