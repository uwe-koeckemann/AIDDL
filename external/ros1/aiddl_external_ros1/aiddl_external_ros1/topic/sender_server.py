from concurrent import futures

import grpc

from aiddl_core.parser import parser
from aiddl_external_grpc_python.sender import SenderServer
from aiddl_external_grpc_python.converter import Converter
import aiddl_external_grpc_python.generated.empty_pb2 as empty_pb2

class TopicSenderServer(SenderServer):
    def __init__(self, port, publisher, f_convert, verbose=False):
        super(TopicSenderServer, self).__init__(port)
        self.pub = publisher
        self.f_convert = f_convert
        self.verbose = verbose
      
    def send(self, request, context):
        if self.verbose:
            print("Sending:", request)
        ros_msg = self.f_convert(request)
        if self.verbose:
            print("Converted to:", ros_msg)
        self.pub.publish(ros_msg)
        return empty_pb2.Empty()
