from concurrent import futures
import time
import math
import logging

import grpc

from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.parser import parser

from aiddl_network.grpc_function import GrpcFunctionLoader
import aiddl_network.aiddl_pb2 as aiddl_pb2
import aiddl_network.aiddl_pb2_grpc as aiddl_pb2_grpc

LOADER_URI = Symbolic("org.aiddl.network.register-remote-function")


class AiddlServicer(aiddl_pb2_grpc.AiddlServicer):
    def __init__(self, port, freg, allow_remote_registry=False):
        self.port = port
        self.freg = freg
        if allow_remote_registry:
            self.freg.add_function(LOADER_URI,
                                   GrpcFunctionLoader(self.freg))

    def FunctionCall(self, request, context):
        uri = parser.parse_term(request.function_uri)
        args = parser.parse_term(request.arg)
        # print("Request to call:", uri, "with:", args)
        f = self.freg.get_function(uri)
        r = f.apply(args)
        # print("Result:", r)
        result = aiddl_pb2.FunctionCallResult(status=0, result=str(r))
        return result

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        aiddl_pb2_grpc.add_AiddlServicer_to_server(
            self, self.server)
        self.server.add_insecure_port('0.0.0.0:%d' % (self.port))
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
