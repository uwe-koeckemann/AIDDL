from concurrent import futures
from abc import ABC, abstractmethod

import grpc

from aiddl_external_grpc_python.generated import function_pb2_grpc, aiddl_pb2


class FunctionServer(ABC, function_pb2_grpc.FunctionServicer):
    def __init__(self, port):
        self.port = port

    @abstractmethod
    def call(self, request, context):
        pass

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        function_pb2_grpc.add_FunctionServicer_to_server(self, self.server)
        self.server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()