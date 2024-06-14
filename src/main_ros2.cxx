#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "MotiveSM.h"
#include "MotiveUtils.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

static const char* TAG = "main";
static const char* TOPIC_NAME_BASE = "/optitrack";

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received signal %d", signum);
    std::exit(signum);
}

static std::shared_ptr<MotiveConfig> GetConfig(const std::string& arg_config_path) {
    MotiveConfig config;
    config.ReadFromYaml(arg_config_path);
    if (config.IsValid()) {
        LOGI(TAG, "Read config file successfully");
    }
    return std::make_shared<MotiveConfig>(config);
}

class OptitrackConfigNode : public rclcpp::Node {
public:
    OptitrackConfigNode() : Node("optitrack_config") {
        this->declare_parameter<std::string>("config_path", "config.yaml");
        this->get_parameter("config_path", this->config_path_);
    }
    std::string GetConfigPath() {
        return config_path_;
    }
private:
    std::string config_path_;
};

class OptitrackPublisher : public rclcpp::Node {
public:
    OptitrackPublisher(std::shared_ptr<MotiveConfig> config_in) : Node("optitrack_publisher"), count_(0), last_rd_cnt_(0), sm_(config_in) {
        // publisher_ = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME_BASE, 10);

        sm_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        publish_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        sm_timer_ = this->create_wall_timer(500ms, std::bind(&OptitrackPublisher::sm_timer_callback, this), sm_callback_group_);
        publish_timer_ = this->create_wall_timer(20ms, std::bind(&OptitrackPublisher::publish_timer_callback, this), publish_callback_group_);
    }

private:
    void sm_timer_callback() {
        LOGI(TAG, "sm_callback_called");
        sm_.Loop();
        if (sm_.GetState()->getType() == MotiveStateEnum::CONNECTED) {
            auto curr_data_description_ = MotiveUtils::GetDataDescription();

            std::map<std::string, bool> all_rb;
            std::vector<std::string> rb_to_add;
            std::vector<std::string> rb_to_remove;
            for (auto& kv : curr_data_description_) {
                all_rb[kv.second] = 1;
                if (publisher_map_.find(kv.second) == publisher_map_.end()) {
                    rb_to_add.push_back(kv.second);
                }
            }
            for (auto& kv : publisher_map_) {
                if (all_rb.find(kv.first) == all_rb.end()) {
                    rb_to_remove.push_back(kv.first);
                }
            }

            std::lock_guard<std::mutex> lock(publisher_mod_mutex_);
            for (auto& rb : rb_to_remove) this->removeRigidBody(rb);
            for (auto& rb : rb_to_add) this->addRigidBody(rb);
            cache_data_description_ = curr_data_description_;
            return;
        }
    }

    void publish_timer_callback() {
        std::lock_guard<std::mutex> lock(publisher_mod_mutex_);
        // auto message = std_msgs::msg::String();
        // message.data = "Hello, world! " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        auto buffer = sm_.GetBuffer();
        auto curr_rd_cnt = buffer->GetCounter();
        if (curr_rd_cnt <= last_rd_cnt_ + 1) {
            return;
        }

        for (auto idx = last_rd_cnt_ + 1; idx < curr_rd_cnt; idx++) {
            auto [data_ptr, curr_idx, err] = sm_.GetBuffer()->Peek((int64_t)idx);
            for (auto rb_idx = 0; rb_idx < data_ptr->nRigidBodies; rb_idx++) {
                auto rb_data = data_ptr->RigidBodies[rb_idx];
                auto rb_name = cache_data_description_.find(rb_data.ID);
                if (rb_name != cache_data_description_.end()) {
                    auto rb_name_str = (*rb_name).second;
                    auto pose = geometry_msgs::msg::PoseStamped();
                    pose.header.stamp = rclcpp::Time(data_ptr->CameraMidExposureTimestamp * 1000);
                    pose.header.frame_id = "optitrack";
                    publisher_map_[rb_name_str]->publish(pose);
                }
            }
        }
    }

    void addRigidBody(const std::string& rigid_body_id) {
        std::string curr_topic = std::string(TOPIC_NAME_BASE) + "/" + rigid_body_id;
        publisher_map_[rigid_body_id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(curr_topic, 10);
    }

    void removeRigidBody(const std::string& rigid_body_id) {
        publisher_map_.erase(rigid_body_id);
    }

    size_t count_;
    int64_t last_rd_cnt_;
    MotiveStateMachine sm_;

    std::mutex publisher_mod_mutex_{};
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publisher_map_{};
    std::map<int, std::string> cache_data_description_{};

    rclcpp::CallbackGroup::SharedPtr sm_callback_group_;
    rclcpp::CallbackGroup::SharedPtr publish_callback_group_;

    rclcpp::TimerBase::SharedPtr sm_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);

    // Loading Configuration
    auto cfg_node = std::make_shared<OptitrackConfigNode>();
    LOGI(TAG, "Loading configuration from %s", cfg_node->GetConfigPath().c_str());
    auto cfg = GetConfig(cfg_node->GetConfigPath());
    if (!cfg->IsValid()) {
        LOGE(TAG, "Failed to read config file");
        throw std::runtime_error("Failed to read config file");
    }

    // Define node
    auto node = std::make_shared<OptitrackPublisher>(cfg);

    // Use executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}