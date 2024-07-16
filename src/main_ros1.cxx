// //
// // Created by liyutong on 2024/6/16.
// //
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "MotiveSM.h"

// static const char* TAG = "main";

// static std::shared_ptr<MotiveConfig> GetConfig(const std::string& arg_config_path) {
//     MotiveConfig config;
//     config.ReadFromYaml(arg_config_path);
//     if (config.IsValid()) {
//         LOGI(TAG, "Read config file successfully");
//     }
//     return std::make_shared<MotiveConfig>(config);
// }

// int main() {
//      MotiveStateMachine sm(GetConfig("./config.yaml"));
//     if (!sm.GetConfig().IsValid()) {
//         LOGE(TAG, "Failed to read config file");
//         return -1;
//     }
//     while (true) {
//         sm.Loop();
//     }
// }

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "MotiveSM.h"
#include "MotiveUtils.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

static const char* TAG = "main";
static const char* TOPIC_NAME_BASE = "/optitrack";

void signalHandler(int signum) {
    ROS_INFO("Received signal %d", signum);
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

class OptitrackConfigNode : public ros::NodeHandle {
public:
    OptitrackConfigNode() : NodeHandle("optitrack_config") {
        this->param<std::string>("config_path", config_path_, "config.yaml");
    }
    std::string GetConfigPath() {
        return config_path_;
    }
private:
    std::string config_path_;
};

class OptitrackPublisher {
public:
    OptitrackPublisher(std::shared_ptr<MotiveConfig> config_in) : count_(0), last_publish_cnt(0), last_visualization_cnt_(0), sm_(config_in) {
        ros::NodeHandle n;
        sm_timer_ = n.createTimer(ros::Duration(0.5), &OptitrackPublisher::sm_timer_callback, this);
        publish_timer_ = n.createTimer(ros::Duration(0.02), &OptitrackPublisher::publish_timer_callback, this);
        visualization_timer_ = n.createTimer(ros::Duration(0.02), &OptitrackPublisher::visualization_timer_callback, this);
    }

private:
    void sm_timer_callback(const ros::TimerEvent&) {
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

            std::lock_guard<std::mutex> lck1(publisher_mod_mutex_);
            std::lock_guard<std::mutex> lck2(visualization_mod_mutex_);
            for (auto& rb : rb_to_remove) this->removeRigidBody(rb);
            for (auto& rb : rb_to_add) this->addRigidBody(rb);
            cache_data_description_ = curr_data_description_;
            return;
        }
    }

    void publish_timer_callback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lck1(publisher_mod_mutex_);
        auto buffer = sm_.GetBuffer();
        auto curr_rd_cnt = buffer->GetCounter();
        if (curr_rd_cnt <= last_publish_cnt + 1) {
            return;
        }

        for (auto idx = last_publish_cnt + 1; idx < curr_rd_cnt; idx++) {
            std::unique_ptr<sFrameOfMocapData> data_ptr;
            RingBufferErr err;
            std::tie(data_ptr, std::ignore, err) = sm_.GetBuffer()->Peek((int64_t)idx);
            for (auto rb_idx = 0; rb_idx < data_ptr->nRigidBodies; rb_idx++) {
                auto rb_data = data_ptr->RigidBodies[rb_idx];
                auto rb_name = cache_data_description_.find(rb_data.ID);
                if (rb_name != cache_data_description_.end()) {
                    auto rb_name_str = (*rb_name).second;
                    // Publish PoseStamped
                    auto pose = geometry_msgs::PoseStamped();
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "world";
                    pose.pose.position.x = rb_data.x;
                    pose.pose.position.y = rb_data.y;
                    pose.pose.position.z = rb_data.z;
                    pose.pose.orientation.x = rb_data.qx;
                    pose.pose.orientation.y = rb_data.qy;
                    pose.pose.orientation.z = rb_data.qz;
                    pose.pose.orientation.w = rb_data.qw;
                    // LOGD(TAG, "Pose for %s is x:%.2f, y:%.2f, z:%.2f, qx:%.2f, qy:%.2f, qz:%.2f, qw:%.2f", rb_name_str.c_str(), rb_data.x, rb_data.y, rb_data.z, rb_data.qx, rb_data.qy, rb_data.qz, rb_data.qw);
                    publisher_map_[rb_name_str].publish(pose);
                }
            }
        }
        last_publish_cnt = curr_rd_cnt-1;
    }

    void visualization_timer_callback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lck2(visualization_mod_mutex_);
        auto buffer = sm_.GetBuffer();
        auto curr_rd_cnt = buffer->GetCounter();
        if (curr_rd_cnt <= last_visualization_cnt_ + 1) {
            return;
        }

        for (auto idx = last_visualization_cnt_ + 1; idx < curr_rd_cnt; idx++) {
            std::unique_ptr<sFrameOfMocapData> data_ptr;
            RingBufferErr err;
            std::tie(data_ptr, std::ignore, err) = sm_.GetBuffer()->Peek((int64_t)idx);
            for (auto rb_idx = 0; rb_idx < data_ptr->nRigidBodies; rb_idx++) {
                auto rb_data = data_ptr->RigidBodies[rb_idx];
                auto rb_name = cache_data_description_.find(rb_data.ID);
                if (rb_name != cache_data_description_.end()) {
                    auto rb_name_str = (*rb_name).second;
                    // Publish tf pose for rviz visualization.
                    double fractional_part, integer_part;
                    fractional_part = modf(data_ptr->fTimestamp, &integer_part);

                    geometry_msgs::TransformStamped transformStamped;
                    // transformStamped.header.stamp = ros::Time((int32_t)integer_part, (int32_t)(fractional_part * 1e9));
                    transformStamped.header.stamp = ros::Time::now();

                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = rb_name_str;
                    transformStamped.transform.translation.x = rb_data.x;
                    transformStamped.transform.translation.y = rb_data.y;
                    transformStamped.transform.translation.z = rb_data.z;
                    transformStamped.transform.rotation.x = rb_data.qx;
                    transformStamped.transform.rotation.y = rb_data.qy;
                    transformStamped.transform.rotation.z = rb_data.qz;
                    transformStamped.transform.rotation.w = rb_data.qw;

                    // LOGD(TAG, "Pose for %s is x:%.2f, y:%.2f, z:%.2f, qx:%.2f, qy:%.2f, qz:%.2f, qw:%.2f", rb_name_str, rb_data.x, rb_data.y, rb_data.z, rb_data, qx, rb_data.qy, rb_data.qz, rb_data.qw);
                    visualization_map_[rb_name_str]->sendTransform(transformStamped);
                }
            }
        }
        last_visualization_cnt_ = curr_rd_cnt-1;
    }



    void addRigidBody(const std::string& rigid_body_id) {
        std::string curr_topic = std::string(TOPIC_NAME_BASE) + "/" + rigid_body_id;
        std::string curr_viz_topic = std::string(TOPIC_NAME_BASE) + "/rviz/" + rigid_body_id;
        ros::NodeHandle n;
        publisher_map_[rigid_body_id] = n.advertise<geometry_msgs::PoseStamped>(curr_topic, 10);
        visualization_map_[rigid_body_id] = std::make_unique<tf2_ros::TransformBroadcaster>();
    }

    void removeRigidBody(const std::string& rigid_body_id) {
        publisher_map_.erase(rigid_body_id);
        visualization_map_.erase(rigid_body_id);
    }

    size_t count_;
    int64_t last_publish_cnt;
    int64_t last_visualization_cnt_;
    MotiveStateMachine sm_;
    
    std::mutex publisher_mod_mutex_{};
    std::mutex visualization_mod_mutex_{};
    std::map<std::string, ros::Publisher> publisher_map_{};
    std::map<std::string, std::unique_ptr<tf2_ros::TransformBroadcaster>> visualization_map_{};
    std::map<int, std::string> cache_data_description_{};

    ros::Timer sm_timer_;
    ros::Timer publish_timer_;
    ros::Timer visualization_timer_;
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "optitrack_publisher");

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
    ros::spin();

    return 0;
}