//
// Created by liyutong on 2024/6/4.
//
#include <cstdio>
#include <thread>
#include "MotiveSM.h"

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

#include "NatNetTypes.h"

#include "debug.h"
#include "ring_buf.hpp"
#include "../lib/tracker_packet/include/server_impl.h"

ABSL_FLAG(std::string, config, "", "Path to the config file");
ABSL_FLAG(uint16_t, grpc_port, 50051, "gRPC port to listen on");

static const char *TAG = "main";

static std::shared_ptr<MotiveConfig> GetConfig(const std::string& arg_config_path) {
    std::vector<std::string> search_path;
    if (arg_config_path.empty()) {
        LOGW(TAG, "Error: --config parameter is missing, using default config file path");
        auto default_config_path = MotiveConfig::GetYamlSearchPath();
        search_path.insert(search_path.begin(), default_config_path.begin(), default_config_path.end());
    } else {
        LOGI(TAG, "Using config file: %s", arg_config_path.c_str());
        search_path.push_back(arg_config_path);
    }
    MotiveConfig config;
    for (const auto &config_path : search_path) {
        if (config.ReadFromYaml(config_path)) {
            break;
        } else {
            LOGD(TAG, "Failed to read config file: %s", config_path.c_str());
        }
    }
    if (!config.IsValid()) {
        LOGE(TAG, "Failed to read config file");
    } else {
        LOGI(TAG, "Read config file successfully");
    }
    return std::make_shared<MotiveConfig>(config);
}

int main(int argc, char* argv[]) {
    // Parse command line flags
    absl::ParseCommandLine(argc, argv);
    std::string arg_config_path = absl::GetFlag(FLAGS_config);
    uint16_t arg_grpc_port = absl::GetFlag(FLAGS_grpc_port);

    MotiveStateMachine sm(GetConfig(arg_config_path));
    if (!sm.GetConfig().IsValid()) {
        LOGE(TAG, "Failed to read config file");
        return -1;
    }

    TrackerServiceImpl service;
    service.SetBuffer(sm.GetBuffer());
    service.Build();

    std::thread grpc_thread([&service, &arg_grpc_port] {
        // Start the gRPC server
        service.RunForever(arg_grpc_port);
    });

    std::thread natnet_thread([&sm] {
        // Start the state machine
        while (true) {
            sm.Loop();
        }
    });
    // loop until ctrl-c is pressed
    grpc_thread.join();
    natnet_thread.join();
}