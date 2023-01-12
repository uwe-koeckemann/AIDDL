import grpc

import aiddl_external_grpc_python.generated.sender_pb2_grpc as sender_pb2_grpc
from aiddl_external_grpc_python.converter import Converter


class FunctionClient:
    def __init__(self, host, port, container):
        channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = sender_pb2_grpc.SenderStub(channel)
        self.converter = Converter(container)

    def __call__(self, x):
        self.converter.pb2aiddl(self.stub.call(self.converter.aiddl2pb(x)))
