import grpc

from aiddl_core.representation.sym import Sym
from aiddl_core.representation.funref import FunRef
import aiddl_core.parser.parser as parser

import aiddl_external_grpc_python.generated.container_pb2 as container_pb2
import aiddl_external_grpc_python.generated.container_pb2_grpc as container_pb2_grpc
from aiddl_external_grpc_python.converter import Converter


class GrpcFunction:
    def __init__(self, host, port, function_uri, container):
        self.host = host
        self.port = port
        self.converter = Converter(container)
        self.function_uri = str(function_uri)

    def __call__(self, args):
        with grpc.insecure_channel('%s:%d' % (self.host, self.port)) as channel:
            stub = container_pb2_grpc.ContainerStub(channel)
            # print("-------------- Request --------------")
            uri = self.function_uri
            arg = self.converter.aiddl2pb(args)
            req = container_pb2.FunctionCallRequest()
            req.function_uri = uri
            req.arg.CopyFrom(arg)
            answer = stub.FunctionCall(req) # Execute request
            # print("-------------- Answer  --------------")
            # print(answer)
            return self.converter.pb2aiddl(answer.result)


class GrpcFunctionLoader:
    def __init__(self, container):
        self.container = container

    def __call__(self, args):
        uri = args[Sym("uri")]
        host = args[Sym("host")].string_value()
        port = args[Sym("port")].int_value()
        f = GrpcFunction(host, port, uri, self.container)
        self.F.add_function(uri, f)
        return FunRef(uri, self.container.fun_ref)
