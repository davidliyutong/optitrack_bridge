//
// Created by liyutong on 2024/6/13.
//

#include <iostream>
#include <memory>
#include <string>

#include "absl/strings/str_format.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "server_impl.h"
#include "debug.h"
#include "../../MotiveSM/include/MotiveUtils.h"
#include "time_utils.hpp"


static const char *TAG = "grpc_server";

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using tracker::TrackerService;
using tracker::TrackerPacketRequest;
using tracker::TrackerPacketArrayStreamResponse;

// Logic and data behind the server's behavior.
Status TrackerServiceImpl::GetPacketArrayStream(ServerContext *context, const TrackerPacketRequest *request, grpc::ServerWriter<tracker::TrackerPacketArrayStreamResponse> *reactor) {
    // Send stream data to client
    std::unique_ptr<tracker::TrackerPacketArrayStreamResponse> response =std::make_unique<tracker::TrackerPacketArrayStreamResponse>() ;
    std::vector<int> rb_filters;
    auto cacheDataDescription = MotiveUtils::GetDataDescription(); // ID->NameMapping
    if (!request->rigid_body_ids().empty()) {
        for (auto &rb_id_pair : cacheDataDescription) {
            if (std::find(request->rigid_body_ids().begin(), request->rigid_body_ids().end(), rb_id_pair.second) != request->rigid_body_ids().end()) {
                rb_filters.push_back(rb_id_pair.first);
            }
        }
    }
    LOGI(TAG, "Received request: %s", request->DebugString().c_str());
    LOGI(TAG, "Setting %lld rigid body filters", rb_filters.size());
    auto last_rd_cnt = buffer->GetCounter() - 1;

    auto ref_freq = TimeUtils::GetPerformanceFrequency();
    auto ref_pc = TimeUtils::GetPerformanceCounter();
    auto ref_unix = TimeUtils::GetUnixTimestampMicroseconds();

    while (!context->IsCancelled()) {
        response->clear_packets();
        response->set_valid(false);
        auto curr_rd_cnt = buffer->GetCounter();
        if (curr_rd_cnt <= last_rd_cnt + 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        for (auto idx = last_rd_cnt + 1; idx < curr_rd_cnt; idx++) {
            response->set_valid(true);

            RingBufferErr err;
            std::unique_ptr<sFrameOfMocapData> data_ptr;
            std::tie(data_ptr, std::ignore, err) = buffer->Peek((int64_t) idx);
            if (err != RingBufferErr::RingBufferErr_OK) {
                response->set_valid(false);
                break;
            }
            if (idx == last_rd_cnt + 1) {
                LOGD(TAG, "Reading from %d", data_ptr->iFrame);
            }

            auto rb_msg = response->add_packets();
            rb_msg->set_sys_ticks(data_ptr->CameraMidExposureTimestamp);
            rb_msg->set_unix_us((data_ptr->CameraMidExposureTimestamp - ref_pc) * 1000000 / ref_freq + ref_unix);
            rb_msg->set_seq(data_ptr->iFrame);

            for (auto rb_idx = 0; rb_idx < data_ptr->nRigidBodies; rb_idx++) {
                auto rb_data = data_ptr->RigidBodies[rb_idx];
                auto rb_name = cacheDataDescription[rb_data.ID];
                if (rb_filters.empty() || std::find(rb_filters.begin(), rb_filters.end(), rb_data.ID) != rb_filters.end()) {
                    auto rb_raw = tracker::TrackerPacketResponseRaw();
                    rb_raw.set_id(rb_name);

                    auto rb_raw_rot = rb_raw.mutable_rotation();
                    rb_raw_rot->set_w(rb_data.qw);
                    rb_raw_rot->set_x(rb_data.qx);
                    rb_raw_rot->set_y(rb_data.qy);
                    rb_raw_rot->set_z(rb_data.qz);

                    auto rb_raw_pos =  rb_raw.mutable_position();
                    rb_raw_pos->set_x(rb_data.x);
                    rb_raw_pos->set_y(rb_data.y);
                    rb_raw_pos->set_z(rb_data.z);

                    rb_msg->mutable_data()->insert({rb_name, rb_raw});
                }
            }
        }
        last_rd_cnt = curr_rd_cnt - 1;

        // send response
        reactor->Write(*response);
    }
    return Status::OK;
}

Status TrackerServiceImpl::GetTimeInfo(::grpc::ServerContext* context, const ::tracker::Empty* request, ::tracker::TimeInfoResponse* response) {
    response->set_frequency(TimeUtils::GetPerformanceFrequency());
    response->set_pc(TimeUtils::GetPerformanceCounter());
    response->set_unix(TimeUtils::GetUnixTimestampMicroseconds());
    return Status::OK;
}

bool TrackerServiceImpl::Build() {
    if (buffer != nullptr) {
        valid = true;
        return true;
    } else {
        return false;
    }
}


void TrackerServiceImpl::RunForever(uint16_t port) {
    if (!valid) {
        LOGE(TAG, "Server is not valid, aborting");
        throw std::runtime_error("Server is not valid, aborting");
    }
    // Listen on the given address without any authentication mechanism.
    std::string server_address = absl::StrFormat("%s:%d", listen_interface, port);
    // Listen on the given address without any authentication mechanism.
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(reinterpret_cast<grpc::Service *>(this));
    // Finally assemble the server.
    server = builder.BuildAndStart();
    LOGI(TAG, "gRPC Server listening on %s", server_address.c_str());
    server->Wait();
}

void TrackerServiceImpl::SetBuffer(std::shared_ptr<RingBuffer<sFrameOfMocapData>> buffer_in) {
    this->buffer = std::move(buffer_in);
}
