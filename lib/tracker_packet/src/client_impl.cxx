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
using tracker::RigidBodyDescriptionArray;

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

std::vector<RigidBodyDescription_t> TrackerClient::GetRigidBodyDescription() {
    TrackerPacketRequest request;

    RigidBodyDescriptionArray response;
    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    stub_->GetRigidBodyDescription(&context, request, &response);

    auto res = std::vector<RigidBodyDescription_t>();
    for (int i = 0; i < response.data_size(); i++) {
        const auto& rb = response.data(i);
        auto rb_desc = RigidBodyDescription_t();
        rb_desc.id = rb.id();
        for (int j = 0; j < rb.markers_size(); j++) {
            const auto& marker = rb.markers(j);
            rb_desc.markers.push_back({marker.x(), marker.y(), marker.z()});
        }
        res.push_back(rb_desc);
    }

    return res;
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
