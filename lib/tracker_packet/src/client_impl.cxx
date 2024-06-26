//
// Created by liyutong on 2024/6/13.
//

#include <grpcpp/channel.h>

#include "client_impl.h"
#include "tracker_packet.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using tracker::TrackerService;
using tracker::TrackerPacketRequest;
using tracker::TrackerPacketArrayStreamResponse;
using tracker::Empty;
using tracker::TimeInfoResponse;

void TrackerClient::GetPacketArrayStream(TrackerClientCallback_t callback, void *callback_context) {
    // Data we are sending to the server.
    TrackerPacketRequest request;

    // Container for the data we expect from the server.
    TrackerPacketArrayStreamResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    auto response = stub_->GetPacketArrayStream(&context, request);

    // read stream api
    while (response->Read(&reply)) {
        callback(&reply, callback_context);
    }
}

TimeInfo_t TrackerClient::GetTimeInfo() {
    Empty request;

    TimeInfoResponse response;
    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    stub_->GetTimeInfo(&context, request, &response);

    // read stream api
    auto res = TimeInfo_t{};
    res.unix = response.unix();
    res.pc = response.pc();
    res.frequency = response.frequency();
    return res;
}