import grpc

import aiddl_external_grpc_python.generated.sender_pb2_grpc as sender_pb2_grpc
from aiddl_external_grpc_python.converter import Converter


class SenderClient:
    def __init__(self, host, port, container):
        channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = sender_pb2_grpc.SenderStub(channel)
        self.converter = Converter(container)

    def Send(self, message):
        self.stub.send(self.converter.aiddl2pb(message))
