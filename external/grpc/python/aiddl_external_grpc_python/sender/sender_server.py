from concurrent import futures
from abc import ABC, abstractmethod

import grpc

import aiddl_external_grpc_python.generated.sender_pb2_grpc as sender_pb2_grpc


class SenderServer(sender_pb2_grpc.SenderServicer):
    def __init__(self, port):
        self.server = None
        self.port = port

    # @abstractmethod
    def Send(self, request, context):
        pass

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        sender_pb2_grpc.add_SenderServicer_to_server(self, self.server)
        self.server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
