AUTHOR = davidliyutong

.PHONY: task.pb.python_client
task.pb.python_client:
	@echo "===========> Generating protobuf files for python"
	@mkdir -p client/python
	@python -m grpc_tools.protoc -I./lib/tracker_packet/manifests \
	    --python_out=./client/python --grpc_python_out=./client/python ./lib/tracker_packet/manifests/tracker_packet.proto