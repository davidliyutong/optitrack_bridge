//
// Created by liyutong on 2024/6/13.
//

#ifndef OPTITRACK_BRIDGE_SERVER_IMPL_H
#define OPTITRACK_BRIDGE_SERVER_IMPL_H

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "absl/strings/str_format.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "server_impl.h"
#include "tracker_packet.grpc.pb.h"
#include "ring_buf.hpp"
#include "NatNetTypes.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using tracker::TrackerService;
using tracker::TrackerPacketRequest;
using tracker::TrackerPacketArrayStreamResponse;

class TrackerServiceImpl final : public TrackerService::Service {
public:
    explicit TrackerServiceImpl(std::string listen_interface_in = "0.0.0.0"): listen_interface(std::move(listen_interface_in)) {}
    Status GetPacketArrayStream(ServerContext* context, const TrackerPacketRequest* request, grpc::ServerWriter<tracker::TrackerPacketArrayStreamResponse>* reactor) override;
    bool Build();
    void RunForever(uint16_t port);
    void SetBuffer(std::shared_ptr<RingBuffer<sFrameOfMocapData>> buffer_in);
private:
    std::shared_ptr<RingBuffer<sFrameOfMocapData>> buffer;
    std::unique_ptr<Server> server;
    std::string listen_interface;
    ServerBuilder builder{};
    bool valid = false;
};

#endif //OPTITRACK_BRIDGE_SERVER_IMPL_H
