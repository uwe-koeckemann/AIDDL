import grpc

from aiddl_core.representation.symbolic import Symbolic
from aiddl_core.representation.function_reference import FunctionReference

import aiddl_core.parser.parser as parser

import aiddl_util.grpc.generated.aiddl_pb2 as aiddl_pb2
import aiddl_util.grpc.generated.aiddl_pb2_grpc as aiddl_pb2_grpc


class GrpcFunction:
    def __init__(self, host, port, function_uri):
        self.host = host
        self.port = port
        self.function_uri = function_uri

    def apply(self, args):
        with grpc.insecure_channel('%s:%d' % (self.host, self.port)) as channel:
            stub = aiddl_pb2_grpc.AiddlStub(channel)
            # print("-------------- Request --------------")
            req = aiddl_pb2.FunctionCallRequest(function_uri=str(self.function_uri), arg=str(args)) # Create request
            answer = stub.FunctionCall(req) # Execute request
            # print("-------------- Answer  --------------")                    
            # print(answer)

            r = parser.parse_term(answer.result)
            return r


class GrpcFunctionLoader:
    def __init__(self, F):
        self.F = F

    def apply(self, args):
        uri = args[Symbolic("uri")]
        host = args[Symbolic("host")].string_value()
        port = args[Symbolic("port")].int_value()
        f = GrpcFunction(host, port, uri)
        self.F.add_function(uri, f)
        return FunctionReference(uri, self.F)
