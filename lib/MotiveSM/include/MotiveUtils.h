//
// Created by liyutong on 2024/6/10.
//

#ifndef OPTITRACK_BRIDGE_MOTIVEUTILS_H
#define OPTITRACK_BRIDGE_MOTIVEUTILS_H

#include <memory>
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include "MotiveConfig.h"

namespace MotiveUtils {
    void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData);
    void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer *pDiscoveredServer, void *pUserContext);
    void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char *msg);
    sNatNetClientConnectParams GetConnectParams(const sNatNetDiscoveredServer &server, const MotiveConfig &config);
    ErrorCode ConnectClient(const std::shared_ptr<NatNetClient>& Client, sNatNetClientConnectParams connectParams, sServerDescription serverDescription, MotiveConfig &config);
    bool DataDescriptionIsValid();
    bool UpdateDataDescriptions(const std::shared_ptr<NatNetClient>& Client, bool printToConsole);
    bool ResetDataDescriptions();
    std::map<int, std::string> GetAssetIDMapping();
    std::shared_ptr<sDataDescriptions> GetDataDescription();
}

#endif //OPTITRACK_BRIDGE_MOTIVEUTILS_H
