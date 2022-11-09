from concurrent import futures

import grpc

import time

from aiddl_core.representation import Sym

from aiddl_external_grpc_python.container.grpc_function import GrpcFunctionLoader
from aiddl_external_grpc_python.converter import Converter

import aiddl_external_grpc_python.generated.container_pb2 as container_pb2
import aiddl_external_grpc_python.generated.container_pb2_grpc as container_pb2_grpc

LOADER_URI = Sym("org.aiddl.network.register-remote-function")


class ContainerServer(container_pb2_grpc.ContainerServicer):
    def __init__(self, port, container, allow_remote_registry=False):
        self.port = port
        self.freg = container.fun_reg
        self.container = container
        self.converter = Converter(container)
        if allow_remote_registry:
            self.freg.add_function(LOADER_URI,
                                   GrpcFunctionLoader(self.freg))

    def FunctionCall(self, request, context):
        uri = Sym(request.function_uri)
        print(f"Calling: {uri}")
        t_start = time.time()
        args = self.converter.pb2aiddl(request.arg)
        print(f"  - converting input: {time.time()-t_start}s")

        f = self.freg.get_function(uri)
        t_start = time.time()
        print(f"  - function call: {time.time()-t_start}s")

        r = f(args)
        result = container_pb2.FunctionCallResult()
        result.status = container_pb2.SUCCESS

        t_start = time.time()
        result_pb = self.converter.aiddl2pb(r)
        print(f"  - converting output: {time.time()-t_start}s")

        t_start = time.time()
        result.result.CopyFrom(result_pb)
        print(f"  - setting output: {time.time()-t_start}s")

        return result

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        container_pb2_grpc.add_ContainerServicer_to_server(
            self, self.server)
        self.server.add_insecure_port('0.0.0.0:%d' % (self.port))
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
