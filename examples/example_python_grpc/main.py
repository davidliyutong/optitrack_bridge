import sys
import os

sys.path.append("./")
sys.path.append("./client/python")

import grpc
from client.python import tracker_packet_pb2, tracker_packet_pb2_grpc
from google.protobuf.json_format import MessageToDict


OPTITRACK_HOST = os.environ.get("OPTITRACK_HOST", "127.0.0.1")
OPTITRACK_PORT = int(os.environ.get("OPTITRACK_PORT", "50051"))
OPTITRACK_DEMO_RIGID_BODY_ID = "demo"


def main():
    channel = grpc.insecure_channel(f"{OPTITRACK_HOST}:{OPTITRACK_PORT}")
    stub = tracker_packet_pb2_grpc.TrackerServiceStub(channel)

    # 创建一个TrackerPacket请求
    request = tracker_packet_pb2.TrackerPacketRequest(rigid_body_ids=[])

    stream = stub.GetPacketArrayStream(request)
    for response in stream:
        print(response.packets[0].data["demo"].position)
        response_json = MessageToDict(response)
        print(response_json['packets'][0]['data'][OPTITRACK_DEMO_RIGID_BODY_ID]['position'])


if __name__ == "__main__":
    main()
