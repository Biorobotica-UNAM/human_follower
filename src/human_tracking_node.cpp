#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class HumanTracker : public rclcpp::Node
{
public:
    HumanTracker() 
    : Node("human_tracking_node"), enable_(false)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING HUMAN TRACKING NODE...");

        // Subs / pubs
        pub_hf_enable_ = this->create_publisher<std_msgs::msg::Bool>("/hri/human_following/enable", 1);
        pub_lf_enable_ =this->create_publisher<std_msgs::msg::Bool>("/hri/leg_finder/enable", 1);
        pub_pt_enable_ = this->create_publisher<std_msgs::msg::Bool>("/enable_continuous", 1);
        
        //pub_person_tracker_enable_ = this->create_publisher<std_msgs::msg::Bool>("/person_tracker/enable", 1);
        //pub_human_detection_enable_ = this->create_publisher<std_msgs::msg::Bool>("/human_detection/enable", 1);
        //pub_yd_enable_ = this->create_publisher<std_msgs::msg::Bool>("/human_detection/enable", 1);

        // Services
        human_tracking_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/hri/human_tracking/enable",
            std::bind(&HumanTracker::callback_enable_tracking, this,
            std::placeholders::_1, std::placeholders::_2));
    }
    private:
        // Subs / pubs
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hf_enable_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_lf_enable_;
        //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_person_tracker_enable_;
        //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_human_detection_enable_;
        //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_yd_enable_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr human_tracking_srv_;

        bool enable_ = false;
        void callback_enable_tracking(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            enable_ = request->data;

            std_msgs::msg::Bool msg;
            msg.data = enable_;
            pub_hf_enable_->publish(msg);
            pub_lf_enable_->publish(msg);
            //pub_person_tracker_enable_->publish(msg);
            pub_human_detection_enable_->publish(msg);
            pub_yd_enable_->publish(msg);

            response->success = true;
            if (enable_)                                
                response->message = "Human tracking enabled";
            else
                response->message = "Human tracking disabled";

            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}