python -m grpc_tools.protoc protobuf/*.proto -I=protobuf --python_out=protobuf/test --proto_path=protobuf --grpc_python_out=protobuf/test
