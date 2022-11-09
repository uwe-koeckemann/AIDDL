# Protocol Buffer


Compile aiddl message and actor service for actor template:

    python -m grpc_tools.protoc protobuf/actor.proto -I=protobuf --python_out=python/actor_template --proto_path=protobuf --grpc_python_out=python/actor_template

    python -m grpc_tools.protoc protobuf/aiddl_msg.proto -I=protobuf --python_out=python/actor_template --proto_path=protobuf --grpc_python_out=python/actor_template
