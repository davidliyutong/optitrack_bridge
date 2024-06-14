//
// Created by liyutong on 2024/6/10.
//

#ifndef OPTITRACK_BRIDGE_MOTIVECONFIG_H
#define OPTITRACK_BRIDGE_MOTIVECONFIG_H

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "debug.h"


class MotiveConfig {
public:
    std::string MotiveServerAddress;
    std::string MotiveMultiCastAddress;
    float MotiveServerFramerate = 0;

    MotiveConfig() = default;
    ~MotiveConfig() = default;
    bool ReadFromYaml(const std::string &filename);
    static std::vector<std::string> GetYamlSearchPath();
    bool IsValid() const { return valid; }
protected:
    bool valid = false;
};

#endif //OPTITRACK_BRIDGE_MOTIVECONFIG_H
