# PowerShell

# Source envfile
. .\envfile.ps1

# Create .tmp directory
if(!(Test-Path -Path .tmp)){
    New-Item -ItemType directory -Path .tmp
}

# Download NatNet SDK for Linux
if(!($env:DEV_NATNET_VERSION)){
    Write-Output "DEV_NATNET_VERSION is not set. Please set it in envfile."
    exit 1
}

if(Test-Path -Path ".tmp\NatNet_SDK_${env:DEV_NATNET_VERSION}.zip"){
    Write-Output "NatNet SDK for Linux already downloaded."
} else {
    Invoke-WebRequest -Uri "https://s3.amazonaws.com/naturalpoint/software/NatNetSDK/NatNet_SDK_${env:DEV_NATNET_VERSION}.zip" -OutFile ".tmp\NatNet_SDK_${env:DEV_NATNET_VERSION}.zip"
}
Expand-Archive -Path ".tmp\NatNet_SDK_${env:DEV_NATNET_VERSION}.zip" -DestinationPath .\third_party -Force