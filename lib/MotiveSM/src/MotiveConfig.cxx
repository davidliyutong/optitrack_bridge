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

#include "MotiveConfig.h"
#include "NatNetTypes.h"

static const char *TAG = "MotiveConfig";

///
/// @brief Reads the configuration from a specified YAML file.
///
/// This function attempts to load the configuration settings from a YAML file specified by
/// the filename parameter. It ensures that the required "motive" section and the
/// "server_address" field are present. If these requirements are not met, the function
/// logs appropriate error messages and sets the valid flag to false. The function also
/// attempts to read the optional "multicast_address" field, defaulting to
/// NATNET_DEFAULT_MULTICAST_ADDRESS if not found.
///
/// @param filename The path to the YAML configuration file.
/// @return True if the configuration was successfully read and is valid, false otherwise.
///
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

    // The config must have a server_address
    try {
        MotiveServerAddress = config["motive"]["server_address"].as<std::string>();
    } catch (YAML::Exception &e) {
        LOGD(TAG, "Failed to parse motive.server_address: %s", e.what());
        MotiveServerAddress = "";
    }

    // if the config does not have a multicast_address, use the default
    try {
        MotiveMultiCastAddress = config["motive"]["multicast_address"].as<std::string>();
    } catch (YAML::Exception &e) {
        LOGD(TAG, "Failed to parse motive.multicast_address: %s", e.what());
        MotiveMultiCastAddress = NATNET_DEFAULT_MULTICAST_ADDRESS;
    }

    return valid;
}

///
/// @brief Determines the home directory of the current user.
///
/// This function retrieves the home directory of the current user. On Windows, it uses
/// the SHGetFolderPathA function to get the path to the profile directory. On Unix-like
/// systems, it uses getpwuid and getuid to get the path to the home directory.
///
/// @return A string representing the path to the home directory, or an empty string if
///         the home directory could not be determined.
///
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

///
/// @brief Generates a list of paths to search for the YAML configuration file.
///
/// This function generates a vector of strings representing the paths to search for the
/// YAML configuration file. The search paths include a system-wide configuration path,
/// a user-specific configuration path within the home directory, and the current working
/// directory.
///
/// @return A vector of strings representing the search paths for the configuration file.
///
std::vector<std::string> MotiveConfig::GetYamlSearchPath() {
    std::vector<std::string> search_path;
    search_path.emplace_back("/etc/rfmocap/config.yaml");
    search_path.push_back(getHomeDirectory() + "/.config/rfmocap/config.yaml");
    search_path.emplace_back("config.yaml");
    return search_path;
}
