#! /bin/bash 

# This script copies protobuf interfaces to the right places and compiles them.
# NOTE: This should only be needed when interfaces change during development. 

rm protobuf/generated/*

python -m grpc_tools.protoc protobuf/*.proto -I=protobuf --python_out=protobuf/generated --proto_path=protobuf --grpc_python_out=protobuf/generated

# These do not define services
rm protobuf/generated/empty_pb2_grpc.py 
rm protobuf/generated/aiddl_pb2_grpc.py 

# Fix relative imports to work in python3
sed -i 's/import aiddl/from . import aiddl/g' protobuf/generated/*.py
sed -i 's/import sender/from . import sender/g' protobuf/generated/*.py
sed -i 's/import receiver/from . import receiver/g' protobuf/generated/*.py
sed -i 's/import actor/from . import actor/g' protobuf/generated/*.py
sed -i 's/import function/from . import function/g' protobuf/generated/*.py
sed -i 's/import empty/from . import empty/g' protobuf/generated/*.py

rm python/aiddl_external_grpc_python/generated/*pb2.py
cp protobuf/generated/*.py python/aiddl_external_grpc_python/generated/

# Remove and copy proto files to scala library
rm scala/src/main/protobuf/actor.proto
rm scala/src/main/protobuf/aiddl.proto
rm scala/src/main/protobuf/container.proto
rm scala/src/main/protobuf/sender.proto
rm scala/src/main/protobuf/receiver.proto
rm scala/src/main/protobuf/function.proto
rm scala/src/main/protobuf/empty.proto
cp protobuf/actor.proto scala/src/main/protobuf/actor.proto 
cp protobuf/aiddl.proto scala/src/main/protobuf/aiddl.proto
cp protobuf/container.proto scala/src/main/protobuf/container.proto  
cp protobuf/sender.proto scala/src/main/protobuf/sender.proto
cp protobuf/receiver.proto scala/src/main/protobuf/receiver.proto
cp protobuf/function.proto scala/src/main/protobuf/function.proto
cp protobuf/empty.proto scala/src/main/protobuf/empty.proto

