//
// Created by liyutong on 2024/6/10.
//

#ifndef OPTITRACK_BRIDGE_MOTIVESM_H
#define OPTITRACK_BRIDGE_MOTIVESM_H


#include <iostream>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "NatNetClient.h"
#include "MotiveConfig.h"
#include "ring_buf.hpp"

enum class MotiveEventEnum {
    INIT,
    CONNECT,
    DISCONNECT,
    HEARTBEAT,
};

enum class MotiveStateEnum {
    UNINITIALIZED,
    DISCONNECTED,
    CONNECTED,
};

static const std::unordered_map<MotiveStateEnum, std::string> MotiveStateEnumMap = {
    {MotiveStateEnum::UNINITIALIZED,
        "UnInitialized"},
    {MotiveStateEnum::DISCONNECTED,
        "Disconnected"},
    {MotiveStateEnum::CONNECTED,
        "Connected"},
};


class MotiveStateMachine;

class MotiveState {
public:
    MotiveState() : type(MotiveStateEnum::UNINITIALIZED) {};

    virtual ~MotiveState() = default;

    virtual bool handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) = 0;

    std::string getName() const {
        return MotiveStateEnumMap.at(type);
    }

    MotiveStateEnum getType() const {
        return type;
    }

protected:
    MotiveStateEnum type;
};

class MotiveUnInitializedState : public MotiveState {
public:
    MotiveUnInitializedState() {
        type = MotiveStateEnum::UNINITIALIZED;
    }

    bool handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) override;
};

class MotiveDisconnectedState : public MotiveState {
public:
    MotiveDisconnectedState() {
        type = MotiveStateEnum::DISCONNECTED;
    }

    bool handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) override;
};

class MotiveConnectedState : public MotiveState {
public:
    MotiveConnectedState() {
        type = MotiveStateEnum::CONNECTED;
    }

    bool handleEvent(MotiveStateMachine &sm, const MotiveEventEnum &event) override;
private:
    std::chrono::time_point<std::chrono::steady_clock> lastCounterFreezeTime{};
    size_t lastUpdatedCounter{};
    bool frozen = false;
};

class MotiveStateMachine {
public:
    explicit MotiveStateMachine(std::shared_ptr<MotiveConfig> config_in)
    : currentState(std::make_shared<MotiveUnInitializedState>()), timeout(false){
        config = std::move(config_in);
        ringBuffer = std::make_shared<RingBuffer<sFrameOfMocapData>>(1024);
        std::copy(MotiveStateEnumMap.begin(), MotiveStateEnumMap.end(), std::inserter(stateMap, stateMap.end()));
    }

    void SetState(const std::shared_ptr<MotiveState> &state);

    std::shared_ptr<MotiveState> GetState();

    void SetClient(const std::shared_ptr<NatNetClient> &client);

    std::shared_ptr<NatNetClient> GetClient() const;

    void GetConfig(const std::shared_ptr<MotiveConfig> &config_in);

    MotiveConfig &GetConfig() const;

    std::shared_ptr<RingBuffer<sFrameOfMocapData>> GetBuffer() const;

    bool IsTimeout() const;

    void SetTimeoutState(bool);

    void Loop();

protected:
    std::shared_ptr<MotiveState> currentState;
    std::unordered_map<MotiveStateEnum, std::string> stateMap;
    std::mutex mutex;
    std::shared_ptr<NatNetClient> natNetClient;
    std::shared_ptr<MotiveConfig> config;
    std::shared_ptr<RingBuffer<sFrameOfMocapData>> ringBuffer;
    bool timeout;

    bool handleEvent(const MotiveEventEnum &event);
};


#endif //OPTITRACK_BRIDGE_MOTIVESM_H
