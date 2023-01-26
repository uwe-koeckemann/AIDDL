from concurrent import futures

import grpc


from aiddl_external_grpc_python.function import FunctionServer
from aiddl_external_grpc_python.generated import function_pb2_grpc, aiddl_pb2


class ServiceCallServer(FunctionServer):
    def __init__(self, port, container, ros_service_proxy, f_in, f_out, verbose=False):
        super(FunctionServer, self).__init__(port)
        self.converter = Converter(container)
        self.ros_service_proxy = ros_service_proxy
        self.f_in = f_in
        self.f_out = f_out

    def Call(self, request, context):
        args = self.f_in(request.aiddl_str)
        out = self.ros_service_proxy(*args)
        answer = self.f_out(out)
        s = str(answer)
        return aiddl_msg_pb2.AiddlStr(aiddl_str=s)

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        function_pb2_grpc.add_FunctionServicer_to_server(self, self.server)
        self.server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
