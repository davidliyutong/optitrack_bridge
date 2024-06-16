#!/bin/bash

set -e

source envfile

mkdir -p .tmp

# Download NatNet SDK for Linux
if [ -z "$DEV_NATNET_VERSION" ]; then
    echo "DEV_NATNET_VERSION is not set. Please set it in envfile."
    exit 1
fi

if [ -f ".tmp/NatNet_SDK_${DEV_NATNET_VERSION}_ubuntu.tar" ]; then
    echo "NatNet SDK for Linux already downloaded."
else
    curl "https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_${DEV_NATNET_VERSION}_ubuntu.tar" -o ".tmp/NatNet_SDK_${DEV_NATNET_VERSION}_ubuntu.tar"
fi

mkdir -p ./third_party/NatNetSDK

tar -xf ".tmp/NatNet_SDK_${DEV_NATNET_VERSION}_ubuntu.tar" -C ./third_party/NatNetSDK