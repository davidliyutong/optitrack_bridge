//
// Created by liyutong on 2024/6/10.
//
#include <chrono>
#include <thread>
#include <typeinfo>

#include "MotiveSM.h"
#include "debug.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include "MotiveUtils.h"

static const char TAG[] = "MotiveSM";

bool MotiveUnInitializedState::handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) {
    if (event == MotiveEventEnum::INIT) {
        unsigned char ver[4];
        NatNet_GetVersion(ver);
        NatNet_SetLogCallback(MotiveUtils::MessageHandler);
        sm.SetClient(std::make_shared<NatNetClient>());
        auto client = sm.GetClient();
        client->SetFrameReceivedCallback(MotiveUtils::DataHandler, sm.GetBuffer().get());
        sm.SetState(std::make_shared<MotiveDisconnectedState>());
        return true;
    } else {
        return false;
    }
}

bool MotiveDisconnectedState::handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) {
    if (event == MotiveEventEnum::CONNECT) {
        // Discover servers
        NatNetDiscoveryHandle discovery;
        std::vector<sNatNetDiscoveredServer> servers;
        NatNet_CreateAsyncServerDiscovery(&discovery, MotiveUtils::ServerDiscoveredCallback, &servers);
        LOGI(TAG, "Discovering servers, wait for 3 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        NatNet_FreeAsyncServerDiscovery(discovery);

        if (servers.empty()) {
            LOGW(TAG, "No server discovered");
            return false;
        }

        MotiveConfig &config = sm.GetConfig();
        auto client = sm.GetClient();
        bool connected = false;
        for (auto &server: servers) {
            if (strcmp(server.serverAddress, config.MotiveServerAddress.c_str()) == 0) {
                LOGI(TAG, "Using server: %s", server.serverAddress);
                auto ret = MotiveUtils::ConnectClient(client, MotiveUtils::GetConnectParams(server, config), server.serverDescription, config);
                if (ret != ErrorCode_OK) {
                    LOGE(TAG, "Error initializing client. Error code: %d.", ret);
                    continue;
                } else {
                    LOGI(TAG, "Client initialized and ready.");
                    connected = true;
                    break;
                }
            } else {
                LOGI(TAG, "Found server %s but will not use", server.serverAddress);
                continue;
            }
        }

        if (!connected) return false;
        bool ret = MotiveUtils::UpdateDataDescriptions(client, true);
        if (!ret) {
            LOGE(TAG, "Unable to retrieve Data Descriptions from Motive.");
        } else {
            LOGI(TAG, "Data Descriptions updated.");
        }
        sm.SetState(std::make_shared<MotiveConnectedState>());
        return true;
    } else {
        return false;
    }
}

bool MotiveConnectedState::handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) {
    if (event == MotiveEventEnum::DISCONNECT) {
        auto client = sm.GetClient();
        client->Disconnect();
        MotiveUtils::ResetDataDescriptions();
        sm.SetState(std::make_shared<MotiveDisconnectedState>());
        return true;
    } else if (event == MotiveEventEnum::HEARTBEAT) {
        // check if the data descriptions need to be updated
        auto client = sm.GetClient();
        if (!MotiveUtils::DataDescriptionIsValid()) {
            bool ret = MotiveUtils::UpdateDataDescriptions(client, false);
            if (!ret) {
                LOGE(TAG, "Unable to retrieve Data Descriptions from Motive.");
                sm.SetTimeoutState(true);
            } else {
                LOGI(TAG, "Data Descriptions updated.");
            }
        }

        // check if the counter is updating
        auto currCounter = sm.GetBuffer()->GetCounter();
        if (lastUpdatedCounter >= currCounter) {
            if (frozen) {
                auto duration = std::chrono::steady_clock::now() - lastCounterFreezeTime;
                if (duration > std::chrono::seconds(10)) {
                    sm.SetTimeoutState(true);
                }
            } else {
                frozen = true;
                lastCounterFreezeTime = std::chrono::steady_clock::now();
            }
        } else {
            frozen = false;
            lastUpdatedCounter = currCounter;
            sm.SetTimeoutState(false);
        }
        return true;
    } else {
        return false;
    }
}


void MotiveStateMachine::SetState(const std::shared_ptr<MotiveState> &state) {
    currentState = state;
}

std::shared_ptr<MotiveState> MotiveStateMachine::GetState() {
    return currentState;
}

void MotiveStateMachine::SetClient(const std::shared_ptr<NatNetClient> &client) {
    natNetClient = client;
}

std::shared_ptr<NatNetClient> MotiveStateMachine::GetClient() const {
    return natNetClient;
}

void MotiveStateMachine::GetConfig(const std::shared_ptr<MotiveConfig> &config_in) {
    config = config_in;
}

MotiveConfig &MotiveStateMachine::GetConfig() const {
    return *config;
}

std::shared_ptr<RingBuffer<sFrameOfMocapData>> MotiveStateMachine::GetBuffer() const {
    return ringBuffer;
}


bool MotiveStateMachine::IsTimeout() const {
    return timeout;
}

void MotiveStateMachine::SetTimeoutState(bool state) {
    timeout = state;
}

bool MotiveStateMachine::handleEvent(const MotiveEventEnum &event) {
    auto result = currentState->handleEvent(*this, event);
    return result;
}

void MotiveStateMachine::Loop() {
    std::lock_guard<std::mutex> lock(mutex);
    // log current state
    // notice: use long long int to avoid platform related issues
    LOGD(TAG, "Current state: %s, buffer.counter: %lld, client connection: %d", currentState->getName().c_str(), (long long int)ringBuffer->GetCounter(), !IsTimeout());

    // handle all events
    if (currentState->getType() == MotiveStateEnum::UNINITIALIZED) {
        handleEvent(MotiveEventEnum::INIT);
        goto success;
    }

    if (currentState->getType() == MotiveStateEnum::DISCONNECTED) {
        auto ret = handleEvent(MotiveEventEnum::CONNECT);
        if (!ret) {
            goto fail;
        } else {
            ringBuffer->Reset();
            goto success;
        }
    }

    if (currentState->getType() == MotiveStateEnum::CONNECTED) {
        handleEvent(MotiveEventEnum::HEARTBEAT);
        if (IsTimeout()) {
            handleEvent(MotiveEventEnum::DISCONNECT);
        }
        goto success;
    }

success:
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return;
fail:
    LOGI(TAG, "Previous Transaction Failed, wait for 5 seconds");
    std::this_thread::sleep_for(std::chrono::seconds(5));
}