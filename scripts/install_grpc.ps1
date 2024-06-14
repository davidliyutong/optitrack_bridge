# PowerShell

Set-Location -Path "third_party\grpc"
git submodule update --init --recursive --depth 1

New-Item -ItemType Directory -Path "cmake\build" -Force
Set-Location -Path "cmake\build"
cmake -DgRPC_BUILD_TESTS=OFF `
      -DgRPC_INSTALL=ON `
      -DCMAKE_INSTALL_PREFIX="$(Get-Location)\..\..\install" `
      "..\.."
cmake --build . --config Debug --target install --parallel 16
