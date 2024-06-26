//
// Created by liyutong on 2024/6/10.
//
#include <vector>

#include "MotiveUtils.h"
#include "NatNetClient.h"
#include "debug.h"
#include "ring_buf.hpp"

static const char *TAG = "MotiveUtils";

namespace MotiveUtils {
    static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS; // TODO: make this configurable
    sDataDescriptions *g_pDataDefs = nullptr;
    auto g_xDataDefUpdateTime = std::chrono::steady_clock::now();
    std::map<int, int> g_AssetIDtoAssetDescriptionOrder;
    std::map<int, std::string> g_AssetIDtoAssetName;
    std::mutex g_DataDefMutex{};

    void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData) {
        auto *pBuffer = (RingBuffer<sFrameOfMocapData> *) pUserData;

        // Note : This function is called every 1 / mocap rate ( e.g. 100 fps = every 10 msecs )
        // We don't want to do too much here and cause the network processing thread to get behind,
        // so let's just safely add this frame to our shared  'network' frame queue and return.

        // Note : The 'data' ptr passed in is managed by NatNet and cannot be used outside this function.
        // Since we are keeping the data, we need to make a copy of it.
        std::shared_ptr<sFrameOfMocapData> pDataCopy = std::make_shared<sFrameOfMocapData>();
        NatNet_CopyFrame(data, pDataCopy.get());
        pBuffer->Push(*pDataCopy);
        // TODO: push to buffer
        NatNet_FreeFrame(pDataCopy.get());
    }

    void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer *pDiscoveredServer, void *pUserContext) {
        auto discoveredServers = (std::vector<sNatNetDiscoveredServer> *) pUserContext;
        discoveredServers->push_back(*pDiscoveredServer);
    }

    void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char *msg) {
        // Optional: Filter out debug messages
        if (msgType < Verbosity_Info) {
            return;
        }
        static const char *TAG = "NatNetLib";
        switch (msgType) {
            case Verbosity_Debug:
                LOGD(TAG, "%s", msg);
                break;
            case Verbosity_Info:
                LOGI(TAG, "%s", msg);
                break;
            case Verbosity_Warning:
                LOGW(TAG, "%s", msg);
                break;
            case Verbosity_Error:
                LOGE(TAG, "%s", msg);
                break;
            default:
                LOGD(TAG, "[???] %s", msg);
                break;
        }
    }

    sNatNetClientConnectParams GetConnectParams(const sNatNetDiscoveredServer &server, const MotiveConfig &config) {
        sNatNetClientConnectParams connectParams;
        if (server.serverDescription.bConnectionInfoValid) {
            // Build the connection parameters.
            connectParams.connectionType = server.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
            connectParams.serverCommandPort = server.serverCommandPort;
            connectParams.serverDataPort = server.serverDescription.ConnectionDataPort;
            connectParams.serverAddress = server.serverAddress;
            connectParams.localAddress = server.localAddress;
            connectParams.multicastAddress = config.MotiveMultiCastAddress.c_str();
        } else {
            // We're missing some info because it's a legacy server.
            // Guess the defaults and make a best effort attempt to connect.
            connectParams.connectionType = kDefaultConnectionType;
            connectParams.serverCommandPort = server.serverCommandPort;
            connectParams.serverDataPort = 0;
            connectParams.serverAddress = server.serverAddress;
            connectParams.localAddress = server.localAddress;
            connectParams.multicastAddress = nullptr;
        }
        return connectParams;
    }

    ErrorCode ConnectClient(const std::shared_ptr<NatNetClient>& Client, sNatNetClientConnectParams connectParams, sServerDescription serverDescription, MotiveConfig &config) {
        // Disconnect from any previous server (if connected)
        Client->Disconnect();

        // Connect to NatNet server (e.g. Motive)
        int retCode = Client->Connect(connectParams);
        if (retCode != ErrorCode_OK) {
            // Connection failed - print connection error code
            LOGE(TAG, "Unable to connect to server.  Error code: %d.", retCode);
            return ErrorCode_Internal;
        } else {
            // Connection succeeded
            void *pResult;
            int nBytes = 0;
            ErrorCode ret;

            // example : print server info
            memset(&serverDescription, 0, sizeof(serverDescription));
            ret = Client->GetServerDescription(&serverDescription);
            if (ret != ErrorCode_OK || !serverDescription.HostPresent) {
                LOGE(TAG, "Unable to connect to server. Host not present. Exiting.");
                return ErrorCode_Internal;
            }
            LOGI(TAG, "Server application info:");
            printf("----------------------------------\n");
            printf("Application: %s (ver. %d.%d.%d.%d)\n", serverDescription.szHostApp, serverDescription.HostAppVersion[0],
                   serverDescription.HostAppVersion[1], serverDescription.HostAppVersion[2], serverDescription.HostAppVersion[3]);
            printf("NatNet Version: %d.%d.%d.%d\n", serverDescription.NatNetVersion[0], serverDescription.NatNetVersion[1],
                   serverDescription.NatNetVersion[2], serverDescription.NatNetVersion[3]);
            printf("Client IP:%s\n", connectParams.localAddress);
            printf("Server IP:%s\n", connectParams.serverAddress);
            printf("Server Name:%s\n", serverDescription.szHostComputerName);

            // example : get mocap frame rate
            ret = Client->SendMessageAndWait("FrameRate", &pResult, &nBytes);
            if (ret == ErrorCode_OK) {
                float fRate = *((float *) pResult);
                config.MotiveServerFramerate = fRate;
                printf("Mocap Framerate : %3.2f\n", fRate);
            } else {
                printf("Mocap Framerate : ERR\n");
            }
            printf("----------------------------------\n");
        }

        return ErrorCode_OK;
    }

    void UpdateDataToDescriptionMaps(sDataDescriptions *pDataDefs) {
        g_AssetIDtoAssetDescriptionOrder.clear();
        g_AssetIDtoAssetName.clear();
        int assetID = 0;
        std::string assetName;
        int index = 0;

        if (pDataDefs == nullptr || pDataDefs->nDataDescriptions <= 0)
            return;

        for (int i = 0; i < pDataDefs->nDataDescriptions; i++) {
            assetID = -1;
            assetName = "";

            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
                sRigidBodyDescription *pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                assetID = pRB->ID;
                assetName = std::string(pRB->szName);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton) {
                sSkeletonDescription *pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                assetID = pSK->skeletonID;
                assetName = std::string(pSK->szName);

                // Add individual bones
                // skip for now since id could clash with non-skeleton RigidBody ids in our RigidBody lookup table
                /*
                if (insertResult.second == true)
                {
                    for (int j = 0; j < pSK->nRigidBodies; j++)
                    {
                        // Note:
                        // In the DataCallback packet (sFrameOfMocapData) skeleton bones (rigid bodies) ids are of the form:
                        //   parent skeleton ID   : high word (upper 16 bits of int)
                        //   rigid body id        : low word  (lower 16 bits of int)
                        //
                        // In DataDescriptions packet (sDataDescriptions) they are not, so apply the data id format here
                        // for correct lookup during data callback
                        std::pair<std::map<int, std::string>::iterator, bool> insertBoneResult;
                        sRigidBodyDescription rb = pSK->RigidBodies[j];
                        int id = (rb.parentID << 16) | rb.ID;
                        std::string skeletonBoneName = string(pSK->szName) + (":") + string(rb.szName) + string(pSK->szName);
                        insertBoneResult = g_AssetIDtoAssetName.insert(id, skeletonBoneName);
                    }
                }
                */
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
                // Skip markersets for now as they dont have unique id's, but do increase the index
                // as they are in the data packet
                index++;
                continue;
                /*
                sMarkerSetDescription* pDesc = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                assetID = index;
                assetName = pDesc->szName;
                */
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate) {
                sForcePlateDescription *pDesc = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                assetID = pDesc->ID;
                assetName = pDesc->strSerialNo;
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device) {
                sDeviceDescription *pDesc = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                assetID = pDesc->ID;
                assetName = std::string(pDesc->strName);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera) {
                // skip cameras as they are not in the data packet
                continue;
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Asset) {
                sAssetDescription *pDesc = pDataDefs->arrDataDescriptions[i].Data.AssetDescription;
                assetID = pDesc->AssetID;
                assetName = std::string(pDesc->szName);
            }

            if (assetID == -1) {
                printf("\n[SampleClient] Warning : Unknown data type in description list : %d\n", pDataDefs->arrDataDescriptions[i].type);
            } else {
                // Add to Asset ID to Asset Name map
                std::pair<std::map<int, std::string>::iterator, bool> insertResult;
                insertResult = g_AssetIDtoAssetName.insert(std::pair<int, std::string>(assetID, assetName));
                if (!insertResult.second) {
                    printf("\n[SampleClient] Warning : Duplicate asset ID already in Name map (Existing:%d,%s\tNew:%d,%s\n)",
                           insertResult.first->first, insertResult.first->second.c_str(), assetID, assetName.c_str());
                }
            }

            // Add to Asset ID to Asset Description Order map
            if (assetID != -1) {
                std::pair<std::map<int, int>::iterator, bool> insertResult;
                insertResult = g_AssetIDtoAssetDescriptionOrder.insert(std::pair<int, int>(assetID, index++));
                if (!insertResult.second) {
                    printf("\n[SampleClient] Warning : Duplicate asset ID already in Order map (ID:%d\tOrder:%d\n)", insertResult.first->first, insertResult.first->second);
                }
            }
        }

    }

    static void printfBits(uint64_t val, int nBits) {
        for (int i = nBits - 1; i >= 0; i--) {
            printf("%d", (int) (val >> i) & 0x01);
        }
        printf("\n");
    }

    void PrintDataDescriptions(sDataDescriptions *pDataDefs) {
        LOGI(TAG, "Received %d Data Descriptions", pDataDefs->nDataDescriptions);
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++) {
            printf("=====================================\n");
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            printf("-------------------------------------\n");
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
                // MarkerSet
                sMarkerSetDescription *pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("  MarkerSet Name : %s\n", pMS->szName);
                printf("  Marker Count : %d\n", pMS->nMarkers);
                if (pMS->szMarkerNames != nullptr) {
                    printf("  Marker Names :\n");
                    for (int j = 0; j < pMS->nMarkers; j++)
                        printf("    - %s\n", pMS->szMarkerNames[j]);
                }
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
                // RigidBody
                sRigidBodyDescription *pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("  RigidBody Name : %s\n", pRB->szName);
                printf("  RigidBody ID : %d\n", pRB->ID);
                printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                printf("  RigidBody Marker Count : %d\n", pRB->nMarkers);
                printf("  RigidBody Marker Positions : \n");
                if (pRB->MarkerPositions != nullptr && pRB->MarkerRequiredLabels != nullptr) {
                    for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx) {
                        const MarkerData &markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf("    Marker #%d:\n", markerIdx);
                        printf("      Position: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2]);

                        if (markerRequiredLabel != 0) {
                            printf("      Required active label: %d\n", markerRequiredLabel);
                        }
                    }
                }
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton) {
                // Skeleton
                sSkeletonDescription *pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("  Skeleton Name : %s\n", pSK->szName);
                printf("  Skeleton ID : %d\n", pSK->skeletonID);
                printf("  RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                printf("  RigidBody (Bone) Data :\n");
                for (int j = 0; j < pSK->nRigidBodies; j++) {
                    sRigidBodyDescription *pRB = &pSK->RigidBodies[j];
                    printf("    - RigidBody Name : %s\n", pRB->szName);
                    printf("      RigidBody ID : %d\n", pRB->ID);
                    printf("      RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("      Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Asset) {
                // Trained Markerset
                sAssetDescription *pAsset = pDataDefs->arrDataDescriptions[i].Data.AssetDescription;
                printf("  Trained Markerset Name : %s\n", pAsset->szName);
                printf("  Asset ID : %d\n", pAsset->AssetID);

                // Trained Markerset Rigid Bodies
                printf("  Trained Markerset RigidBody (Bone) Count : %d\n", pAsset->nRigidBodies);
                printf("  Trained Markerset RigidBody (Bone) Data :\n");
                for (int j = 0; j < pAsset->nRigidBodies; j++) {
                    sRigidBodyDescription *pRB = &pAsset->RigidBodies[j];
                    printf("    - RigidBody Name : %s\n", pRB->szName);
                    printf("      RigidBody ID : %d\n", pRB->ID);
                    printf("      RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("      Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }

                // Trained Markerset Markers
                printf("  Trained Markerset Marker Count : %d\n", pAsset->nMarkers);
                printf("  Trained Markerset Marker Data :\n");
                for (int j = 0; j < pAsset->nMarkers; j++) {
                    sMarkerDescription marker = pAsset->Markers[j];
                    int modelID, markerID;
                    NatNet_DecodeID(marker.ID, &modelID, &markerID);
                    printf("    - Marker Name : %s\n", marker.szName);
                    printf("      Marker ID   : %d\n", markerID);
                    printf("      Marker Params : ");
                    printfBits(marker.params, sizeof(marker.params) * 8);
                }
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate) {
                // Force Plate
                sForcePlateDescription *pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("  Force Plate ID : %d\n", pFP->ID);
                printf("  Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("  Force Plate Width : %3.2f\n", pFP->fWidth);
                printf("  Force Plate Length : %3.2f\n", pFP->fLength);
                printf("  Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX, pFP->fOriginY, pFP->fOriginZ);
                printf("  Force Plate Corners : \n");
                for (int iCorner = 0; iCorner < 4; iCorner++)
                    printf("    - Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0], pFP->fCorners[iCorner][1], pFP->fCorners[iCorner][2]);
                printf("  Force Plate Type : %d\n", pFP->iPlateType);
                printf("  Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("  Force Plate Channel Count : %d\n", pFP->nChannels);
                for (int iChannel = 0; iChannel < pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device) {
                // Peripheral Device
                sDeviceDescription *pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                printf("  Device Name : %s\n", pDevice->strName);
                printf("  Device Serial : %s\n", pDevice->strSerialNo);
                printf("  Device ID : %d\n", pDevice->ID);
                printf("  Device Channel Count : %d\n", pDevice->nChannels);
                printf("  Device Channel Data : \n");
                for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
                    printf("    - Channel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera) {
                // Camera
                sCameraDescription *pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                printf("  Camera Name : %s\n", pCamera->strName);
                printf("  Camera Position (%3.2f, %3.2f, %3.2f)\n", pCamera->x, pCamera->y, pCamera->z);
                printf("  Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)\n", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            } else {
                // Unknown
                printf("  Unknown data type.\n");
            }
            printf("=====================================\n\n");
        }
    }

    bool DataDescriptionIsValid() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - g_xDataDefUpdateTime);
        return g_pDataDefs != nullptr && duration < std::chrono::seconds(10);
    }

    bool UpdateDataDescriptions(const std::shared_ptr<NatNetClient>& Client, bool printToConsole) {
        std::lock_guard<std::mutex> lock(g_DataDefMutex);
        // release memory allocated by previous in previous GetDataDescriptionList()
        if (g_pDataDefs) {
            NatNet_FreeDescriptions(g_pDataDefs);
        }

        // Retrieve Data Descriptions from Motive
        LOGI(TAG, "Requesting Data Descriptions...");
        int iResult;
        try {
            iResult = Client->GetDataDescriptionList(&g_pDataDefs);
        } catch (...) {
            LOGE(TAG, "Requesting Data Descriptions Error");
        }

        if (iResult != ErrorCode_OK || g_pDataDefs == nullptr) {
            if (g_pDataDefs) {
                try {
                    NatNet_FreeDescriptions(g_pDataDefs);
                } catch (...){
                    LOGE(TAG, "NatNet_FreeDescriptions Error");
                    g_pDataDefs = nullptr;
                }

            }
            return false;
        } else {
            if (printToConsole) {
                PrintDataDescriptions(g_pDataDefs);
            }
        }

        // Update the Asset ID to Asset Name and Asset ID to Asset Description Order maps
        UpdateDataToDescriptionMaps(g_pDataDefs);
        g_xDataDefUpdateTime = std::chrono::steady_clock::now();

        return true;
    }

    bool ResetDataDescriptions() {
        std::lock_guard<std::mutex> lock(g_DataDefMutex);
        if (g_pDataDefs) {
            NatNet_FreeDescriptions(g_pDataDefs);
            g_pDataDefs = nullptr;
            g_AssetIDtoAssetDescriptionOrder.clear();
            g_AssetIDtoAssetName.clear();
            return true;
        }
        return false;
    }

    std::map<int, std::string> GetDataDescription() {
        std::lock_guard<std::mutex> lock(g_DataDefMutex);
        return g_AssetIDtoAssetName;
    }
}
