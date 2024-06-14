//
// Created by liyutong on 2024/6/13.
//

#ifndef OPTITRACK_BRIDGE_CLIENT_IMPL_H
#define OPTITRACK_BRIDGE_CLIENT_IMPL_H

#include <grpcpp/channel.h>

#include "tracker_packet.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using tracker::TrackerService;
using tracker::TrackerPacketRequest;
using tracker::TrackerPacketArrayStreamResponse;

typedef void (*TrackerClientCallback_t)(TrackerPacketArrayStreamResponse* resp, void * context);

class TrackerClient {
public:
    explicit TrackerClient(const std::shared_ptr<Channel> &channel)
        : stub_(TrackerService::NewStub(channel)) {}

    // Assembles the client's payload, sends it and presents the response back
    // from the server.
    void GetPacketArrayStream(TrackerClientCallback_t callback, void * callback_context);
private:
    std::unique_ptr<TrackerService::Stub> stub_;
};


#endif //OPTITRACK_BRIDGE_CLIENT_IMPL_H
