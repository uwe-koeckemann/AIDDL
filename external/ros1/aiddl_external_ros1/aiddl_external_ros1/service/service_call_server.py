from concurrent import futures

import os
import grpc
import rospy

from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.function import FunctionServer
from aiddl_external_grpc_python.generated import function_pb2_grpc

def run_service_call_connector(node_name, service_type, converter_in, converter_out, verbose=False):
    grpc_port = int(os.getenv("GRPC_PORT"))
    service_name = os.getenv("ROS_SERVICE")

    proxy = rospy.ServiceProxy(service_name, service_type)
    rospy.init_node(node_name, anonymous=True)

    print('Creating sender server')
    server = ServiceCallServer(
        grpc_port,
        proxy,
        converter_in,
        converter_out)
    def exit_handler():
        print('Closing down...')
        server.server.stop(2).wait()
        print('Done.')
    atexit.register(exit_handler)

    print('Starting server...')
    server.start()
    print('Running.')
    rospy.spin()


class ServiceCallServer(FunctionServer):
    def __init__(self, port, ros_service_proxy, f_in, f_out, verbose=False):
        super(ServiceCallServer, self).__init__(port)
        self.ros_service_proxy = ros_service_proxy
        self.f_in = f_in
        self.f_out = f_out
        container = Container()
        self.converter = Converter(container)

    def Call(self, request, context):
        args = self.f_in(request)
        out = self.ros_service_proxy(*args)
        answer = self.f_out(out)
        return self.converter.aiddl2pb(answer)

    def start(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        function_pb2_grpc.add_FunctionServicer_to_server(self, self.server)
        self.server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.server.start()

    def wait_for_termination(self):
        self.server.wait_for_termination()
