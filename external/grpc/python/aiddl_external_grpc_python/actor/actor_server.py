from concurrent import futures
from abc import ABC, abstractmethod

import grpc

import aiddl_external_grpc_python.generated.actor_pb2_grpc as actor_pb2_grpc


class ActorServer(ABC, actor_pb2_grpc.ActorServicer):
    def __init__(self, port):
        self.server = None
        self.port = port
        self.next_id = 0

    @abstractmethod
    def IsSupported(self, request, context):
        pass

    @abstractmethod
    def Dispatch(self, request, context):
        pass

    @abstractmethod
    def Status(self, request, context):
        pass

    @abstractmethod
    def Cancel(self, request, context):
        pass

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        actor_pb2_grpc.add_ActorServicer_to_server(
            self, self.server)
        self.server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
