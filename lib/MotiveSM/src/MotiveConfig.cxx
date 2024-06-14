//
// Created by liyutong on 2024/6/10.
//
#include <iostream>
#include <string>

#ifdef _WIN32
#include <Windows.h>
#include <ShlObj.h>
#else
#include <unistd.h>
#include <pwd.h>
#endif

#include "../include/MotiveConfig.h"
#include "NatNetTypes.h"

static const char *TAG = "MotiveConfig";

bool MotiveConfig::ReadFromYaml(const std::string &filename) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(filename);
    } catch (YAML::BadFile &e) {
        LOGD(TAG, "Failed to open config file: %s", e.what());
        MotiveServerAddress = "";
        return false;
    }

    valid = true;

    // The config must have a motive section
    if (!config["motive"]) {
        LOGD(TAG, "Config file does not have a motive section");
        valid = false;
        return valid;
    }

    try {
        MotiveServerAddress = config["motive"]["server_address"].as<std::string>();
    } catch (YAML::Exception &e) {
        LOGD(TAG, "Failed to parse motive.server_address: %s", e.what());
        MotiveServerAddress = "";
    }

    try {
        MotiveMultiCastAddress = config["motive"]["multicast_address"].as<std::string>();
    } catch (YAML::Exception &e) {
        LOGD(TAG, "Failed to parse motive.multicast_address: %s", e.what());
        MotiveMultiCastAddress = NATNET_DEFAULT_MULTICAST_ADDRESS;
    }

    return valid;
}


std::string getHomeDirectory() {
#ifdef _WIN32
    char homeDir[MAX_PATH];
    if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_PROFILE, nullptr, 0, homeDir))) {
        std::string homeDirStr(homeDir);
        return homeDirStr;
    } else {
        std::cerr << "Error: Unable to determine home directory on Windows." << std::endl;
        return "";
    }
#else
    struct passwd* pw = getpwuid(getuid());
    if (pw == nullptr) {
        std::cerr << "Error: Unable to determine home directory on Unix-like system." << std::endl;
        return "";
    }
    return std::string(pw->pw_dir);
#endif
}

std::vector<std::string> MotiveConfig::GetYamlSearchPath() {
    std::vector<std::string> search_path;
    search_path.emplace_back("/etc/rfmocap/config.yaml");
    search_path.push_back(getHomeDirectory() + "/.config/rfmocap/config.yaml");
    search_path.emplace_back("config.yaml");
    return search_path;
}
