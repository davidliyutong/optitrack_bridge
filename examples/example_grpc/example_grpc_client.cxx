/*
 *
 * Copyright 2015 gRPC authors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>
#include <memory>
#include <string>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

#include <grpcpp/grpcpp.h>

#include "client_impl.h"

ABSL_FLAG(std::string, target, "localhost:50051", "Server address");

void Callback(tracker::TrackerPacketArrayStreamResponse *response, void *arg) {
    for (const auto& x : response->packets()) {
        std::cout << "Received: " << x.DebugString() << std::endl;
    }
}

int main(int argc, char** argv) {
    absl::ParseCommandLine(argc, argv);
    // Instantiate the client. It requires a channel, out of which the actual RPCs
    // are created. This channel models a connection to an endpoint specified by
    // the argument "--target=" which is the only expected argument.
    std::string target_str = absl::GetFlag(FLAGS_target);
    // We indicate that the channel isn't authenticated (use of
    // InsecureChannelCredentials()).
    TrackerClient cli(
        grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));
    TimeInfo_t t = cli.GetTimeInfo();
    std::cout << "Unix: " << t.unix << ", PC: " << t.pc << ", Frequency: " << t.frequency << std::endl;
    auto desc = cli.GetRigidBodyDescription();
    for (const auto& x : desc) {
        std::cout << "Rigid body: " << x.id << std::endl;
        for (const auto& y : x.markers) {
            std::cout << "Marker: " << y[0] << ", " << y[1] << ", " << y[2] << std::endl;
        }
    }
    // sleep for 5 seconds
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    cli.GetPacketArrayStream(Callback, nullptr);

    return 0;
}
